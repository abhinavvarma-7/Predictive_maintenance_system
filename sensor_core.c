#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <sys/dispatch.h>
#include <hw/inout.h>
#include <hw/i2c.h>
#include <devctl.h>
#include <unistd.h>
#include <pthread.h>
#include "shared_ipc.h"

#define BCM2711_GPIO_BASE   0xFE200000
#define GPIO_LEN            0xB4
#define DHT_PIN             4
#define VIB_PIN             17
#define GPSET0    0x1C
#define GPCLR0    0x28
#define GPLEV0    0x34
#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B

#define GPIO_MODE_IN(g, ptr)  (*(ptr + ((g)/10)) &= ~(7 << (((g)%10)*3)))
#define GPIO_MODE_OUT(g, ptr) (*(ptr + ((g)/10)) |=  (1 << (((g)%10)*3)))
#define GPIO_SET(g, ptr)      (*(ptr + (GPSET0/4)) = (1 << g))
#define GPIO_CLR(g, ptr)      (*(ptr + (GPCLR0/4)) = (1 << g))
#define GPIO_READ(g, ptr)     (*(ptr + (GPLEV0/4)) & (1 << g))

uint64_t cps;
uint64_t cycle_threshold_50us;
sensor_packet_t current_state;
pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile uint32_t *gpio_ptr;
int i2c_fd;

int wait_for_pin(volatile uint32_t *gpio, int target_state, uint32_t max_retries) {
    uint32_t count = 0;
    while (1) {
        int current_state = GPIO_READ(DHT_PIN, gpio) ? 1 : 0;
        if (current_state == target_state) return 1;
        if (count++ > max_retries) return 0;
    }
}

void init_mpu(int fd) {
    struct { i2c_send_t hdr; uint8_t bytes[2]; } msg;
    msg.hdr.slave.addr = MPU_ADDR;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.len = 2;
    msg.bytes[0] = PWR_MGMT_1; msg.bytes[1] = 0x00;
    devctl(fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL);
}

int read_mpu(int fd, int *out_x, int *out_y, int *out_z) {
    struct { i2c_sendrecv_t hdr; uint8_t buf[6]; } msg;
    msg.hdr.slave.addr = MPU_ADDR;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len = 1; msg.hdr.recv_len = 6;
    msg.buf[0] = ACCEL_XOUT_H;

    if (devctl(fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL) == EOK) {
        *out_x = (int16_t)((msg.buf[0] << 8) | msg.buf[1]);
        *out_y = (int16_t)((msg.buf[2] << 8) | msg.buf[3]);
        *out_z = (int16_t)((msg.buf[4] << 8) | msg.buf[5]);
        return 1;
    }
    return 0;
}

int read_dht11(volatile uint32_t *gpio, int *out_temp, int *out_hum) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint64_t start, end;

    GPIO_MODE_OUT(DHT_PIN, gpio); GPIO_CLR(DHT_PIN, gpio); delay(18);
    GPIO_SET(DHT_PIN, gpio); nanospin_ns(30000); GPIO_MODE_IN(DHT_PIN, gpio);

    struct sched_param param; int policy;
    pthread_getschedparam(pthread_self(), &policy, &param);
    int original_prio = param.sched_priority;
    param.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), policy, &param);

    if (!wait_for_pin(gpio, 0, 20000)) goto restore_prio;
    if (!wait_for_pin(gpio, 1, 20000)) goto restore_prio;
    if (!wait_for_pin(gpio, 0, 20000)) goto restore_prio;

    for (int i = 0; i < 40; i++) {
        if (!wait_for_pin(gpio, 1, 20000)) goto restore_prio;
        start = ClockCycles();
        if (!wait_for_pin(gpio, 0, 20000)) goto restore_prio;
        end = ClockCycles();
        if ((end - start) > cycle_threshold_50us) data[i/8] |= (1 << (7 - (i%8)));
    }

    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        *out_hum = data[0]; *out_temp = data[2];
        param.sched_priority = original_prio;
        pthread_setschedparam(pthread_self(), policy, &param);
        return 1;
    }

restore_prio:
    param.sched_priority = original_prio;
    pthread_setschedparam(pthread_self(), policy, &param);
    return 0;
}

void *hardware_thread(void *arg) {
    while(1) {
        int tx=0, ty=0, tz=0, tt=0, th=0;
        int m_ok = (i2c_fd >= 0) ? read_mpu(i2c_fd, &tx, &ty, &tz) : 0;
        int d_ok = read_dht11(gpio_ptr, &tt, &th);
        int v_ok = GPIO_READ(VIB_PIN, gpio_ptr) ? 1 : 0;

        pthread_mutex_lock(&state_mutex);
        current_state.x = tx; current_state.y = ty; current_state.z = tz;
        current_state.temp = tt; current_state.hum = th;
        current_state.vib = v_ok;
        current_state.mpu_ok = m_ok; current_state.dht_ok = d_ok;
        pthread_mutex_unlock(&state_mutex);

        // SILENT: Removed the debug printf from here!
        delay(500);
    }
    return NULL;
}

int main() {
    printf("[SERVER] Starting QNX Sensor Core (Silent Mode)...\n");

    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("[FATAL ERROR] ThreadCtl failed. Must run as root");
        return -1;
    }

    cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    cycle_threshold_50us = (cps * 50) / 1000000;

    gpio_ptr = (volatile uint32_t *)mmap_device_memory(NULL, GPIO_LEN, PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, BCM2711_GPIO_BASE);
    if (gpio_ptr == MAP_FAILED) return -1;

    i2c_fd = open("/dev/i2c1", O_RDWR);
    if (i2c_fd >= 0) init_mpu(i2c_fd);
    GPIO_MODE_IN(VIB_PIN, gpio_ptr);

    pthread_t hw_tid;
    pthread_create(&hw_tid, NULL, hardware_thread, NULL);

    name_attach_t *attach = name_attach(NULL, ATTACH_NAME, 0);
    if (attach == NULL) return -1;
    printf("[SERVER] IPC Channel Ready: %s\n", ATTACH_NAME);

    sensor_request_t msg;
    int rcvid;

    while (1) {
        rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) break;
        if (rcvid == 0) continue;

        if (msg.msg_type == MSG_TYPE_READ_SENSORS) {
            sensor_packet_t reply_data;
            pthread_mutex_lock(&state_mutex);
            reply_data = current_state;
            pthread_mutex_unlock(&state_mutex);
            MsgReply(rcvid, EOK, &reply_data, sizeof(reply_data));
        } else {
            MsgError(rcvid, ENOSYS);
        }
    }
    return 0;
}
