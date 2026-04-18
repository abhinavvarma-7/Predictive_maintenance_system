#ifndef SHARED_IPC_H_
#define SHARED_IPC_H_

#include <stdint.h>
#include <sys/iomsg.h>

// --- SENSOR CORE IPC ---
#define ATTACH_NAME "qnx_sensor_ipc"
#define MSG_TYPE_READ_SENSORS (_IO_MAX + 1)

typedef struct {
    uint16_t msg_type;
} sensor_request_t;

typedef struct {
    int x, y, z;
    int temp, hum;
    int vib;
    int mpu_ok;
    int dht_ok;
} sensor_packet_t;

// --- ALERT NODE IPC ---
#define ALERT_SERVER_NAME "qnx_alert_ipc"
#define MSG_TYPE_ALERT_DATA (_IO_MAX + 2)

typedef struct {
    uint16_t msg_type;
    int raw_z;
    int raw_x;
    int raw_y;
    int raw_temp;
    int raw_vib;
    float slope_z;
    float slope_temp;
    int alert_level;      // 0 = Nominal, 1 = Warning, 2 = Critical
    char alert_text[128]; // Description of the issue
} alert_packet_t;

typedef struct {
    int status;
} alert_reply_t;

#endif
