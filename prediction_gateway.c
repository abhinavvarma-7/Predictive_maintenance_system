#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include "shared_ipc.h"

#define HISTORY_SIZE 12
#define CRITICAL_Z_SLOPE 5.0
#define CRITICAL_T_SLOPE 1.5

int main() {
    setvbuf(stdout, NULL, _IOLBF, 0);
    setenv("TZ", "IST-5:30", 1);
    tzset();

    system("date 041809202026");

    int sensor_coid, alert_coid;
    printf("[GATEWAY] Booting Predictive Maintenance Edge Node...\n");

    // 1. Connect to the Sensor Core (Input)
    while ((sensor_coid = name_open(ATTACH_NAME, 0)) == -1) {
        printf("Waiting for sensor_core to start...\n");
        sleep(1);
    }

    // 2. Connect to the Alert Node (Output)
    while ((alert_coid = name_open(ALERT_SERVER_NAME, 0)) == -1) {
        printf("Waiting for alert_node to start...\n");
        sleep(1);
    }

    printf("[GATEWAY] Fully Connected. Polling hardware every 5 seconds.\n");

    sensor_request_t req;
    sensor_packet_t reply;
    req.msg_type = MSG_TYPE_READ_SENSORS;

    int h_x[HISTORY_SIZE]={0}, h_y[HISTORY_SIZE]={0}, h_z[HISTORY_SIZE]={0};
    int h_t[HISTORY_SIZE]={0}, h_h[HISTORY_SIZE]={0};
    int time_x[HISTORY_SIZE]={0};
    for(int i=0; i<HISTORY_SIZE; i++) time_x[i] = i;
    int count = 0;

    while(1) {
        if (MsgSend(sensor_coid, &req, sizeof(req), &reply, sizeof(reply)) != -1) {

            time_t rawtime;
            struct tm * timeinfo;
            char time_buffer[100] = "WAITING FOR CLOCK SYNC";

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            if (timeinfo != NULL) {
                strftime(time_buffer, sizeof(time_buffer), "%d-%m-%Y %I:%M:%S %p", timeinfo);
            }

            printf("\n======================================================\n");
            printf("  SENSOR ACQUISITION [%s] \n", time_buffer);
            printf("======================================================\n");
            printf("MPU-6500 Accel -> X: %6d | Y: %6d | Z: %6d\n", reply.x, reply.y, reply.z);
            printf("DHT11 Sensor   -> Temp: %d C | Humidity: %d %%\n", reply.temp, reply.hum);
            printf("Vibration Pin  -> %s\n", reply.vib ? "VIBRATION DETECTED (SHOCK)" : "Quiet");
            printf("------------------------------------------------------\n");

            // --- PREDICTIVE MATH ---
            float slope_x = 0, slope_y = 0, slope_z = 0, slope_temp = 0;

            if (count < HISTORY_SIZE) {
                h_x[count] = reply.x; h_y[count] = reply.y; h_z[count] = reply.z;
                h_t[count] = reply.temp; h_h[count] = reply.hum;
                count++;
                printf("[PREDICTOR] Calibrating baseline: %d/%d samples...\n", count, HISTORY_SIZE);
            } else {
                for(int i = 0; i < HISTORY_SIZE - 1; i++) {
                    h_x[i] = h_x[i+1]; h_y[i] = h_y[i+1]; h_z[i] = h_z[i+1];
                    h_t[i] = h_t[i+1]; h_h[i] = h_h[i+1];
                }
                h_x[HISTORY_SIZE-1] = reply.x; h_y[HISTORY_SIZE-1] = reply.y; h_z[HISTORY_SIZE-1] = reply.z;
                h_t[HISTORY_SIZE-1] = reply.temp; h_h[HISTORY_SIZE-1] = reply.hum;

                long sum_x=0, sum_xx=0;
                long sum_y_x=0, sum_y_y=0, sum_y_z=0, sum_y_t=0;
                long sum_xy_x=0, sum_xy_y=0, sum_xy_z=0, sum_xy_t=0;

                for(int i=0; i<HISTORY_SIZE; i++) {
                    sum_x += time_x[i]; sum_xx += (time_x[i]*time_x[i]);
                    sum_y_x += h_x[i]; sum_xy_x += (time_x[i]*h_x[i]);
                    sum_y_y += h_y[i]; sum_xy_y += (time_x[i]*h_y[i]);
                    sum_y_z += h_z[i]; sum_xy_z += (time_x[i]*h_z[i]);
                    sum_y_t += h_t[i]; sum_xy_t += (time_x[i]*h_t[i]);
                }

                float denom = (float)((HISTORY_SIZE * sum_xx) - (sum_x * sum_x));
                slope_x = (float)((HISTORY_SIZE * sum_xy_x) - (sum_x * sum_y_x)) / denom;
                slope_y = (float)((HISTORY_SIZE * sum_xy_y) - (sum_x * sum_y_y)) / denom;
                slope_z = (float)((HISTORY_SIZE * sum_xy_z) - (sum_x * sum_y_z)) / denom;
                slope_temp = (float)((HISTORY_SIZE * sum_xy_t) - (sum_x * sum_y_t)) / denom;

                printf("[PREDICTOR] Trends -> X: %+.2f | Y: %+.2f | Z: %+.2f | Temp: %+.2f\n", slope_x, slope_y, slope_z, slope_temp);
            }

            // ==========================================================
            // NEW: SEND DATA TO ALERT NODE
            // ==========================================================
            alert_packet_t alert_msg;
            alert_reply_t alert_rep;

            alert_msg.msg_type = MSG_TYPE_ALERT_DATA;
            alert_msg.raw_z = reply.z;
            alert_msg.raw_x = reply.x;   // ← add
            alert_msg.raw_y = reply.y;
            alert_msg.raw_temp = reply.temp;
            alert_msg.raw_vib = reply.vib;
            alert_msg.slope_z = slope_z;
            alert_msg.slope_temp = slope_temp;

            // Determine the Alert Level
            alert_msg.alert_level = 0; // Default to Nominal
            strcpy(alert_msg.alert_text, "System Nominal");

            if (reply.vib == 1) {
                alert_msg.alert_level = 2; // Critical
                strcpy(alert_msg.alert_text, "Immediate physical shock detected!");
                printf("🚨 CRITICAL ALARM: %s\n", alert_msg.alert_text);
            } else if (count == HISTORY_SIZE && (slope_z > CRITICAL_Z_SLOPE || slope_temp > CRITICAL_T_SLOPE)) {
                alert_msg.alert_level = 1; // Warning
                strcpy(alert_msg.alert_text, "Predictive Degradation Warning!");
                printf("⚠️ PREDICTIVE WARNING: Trend exceeds safe thresholds!\n");
            }

            // Fire the IPC message over to the Alert Node
            if (MsgSend(alert_coid, &alert_msg, sizeof(alert_msg), &alert_rep, sizeof(alert_rep)) == -1) {
                printf("[GATEWAY] Error: Could not reach Alert Node.\n");
            }

            printf("======================================================\n");

        } else {
            perror("IPC Error: Connection to Sensor Core lost");
            name_close(sensor_coid);
            while ((sensor_coid = name_open(ATTACH_NAME, 0)) == -1) sleep(1);
        }

        sleep(5);
    }

    name_close(sensor_coid);
    name_close(alert_coid);
    return 0;
}
