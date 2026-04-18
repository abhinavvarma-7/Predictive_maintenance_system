#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "shared_ipc.h"

//#define DASHBOARD_IP "10.40.1.120"
#define D_IP "10.0.0.2"
#define D_PORT 5005

int main(void) {
    
    name_attach_t *attach = name_attach(NULL, ALERT_SERVER_NAME, 0);
    if (attach == NULL) {
        perror("[FATAL] name_attach failed. Alert Node already running?");
        return EXIT_FAILURE;
    }

    
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(D_PORT);
    inet_pton(AF_INET, D_IP, &servaddr.sin_addr);

    alert_packet_t msg;
    alert_reply_t reply = {EOK};
    char json_payload[512];

    printf("[ALERT NODE] Online. Waiting for Predictor triggers...\n");

    while (1) {
        
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);

        if (rcvid > 0 && msg.msg_type == MSG_TYPE_ALERT_DATA) {

            
            MsgReply(rcvid, EOK, &reply, sizeof(reply));

           
            if (msg.alert_level == 2) {
                printf("\n[ALERT NODE] CRITICAL: %s\n", msg.alert_text);
            } else if (msg.alert_level == 1) {
                printf("\n[ALERT NODE] WARNING: %s\n", msg.alert_text);
            } else {
                printf("[ALERT NODE] System nominal. Z-Slope: %.2f | T-Slope: %.2f\n", msg.slope_z, msg.slope_temp);
            }

            
            snprintf(json_payload, sizeof(json_payload),
                "{\"x_raw\": %d, \"y_raw\": %d, \"z_raw\": %d, "
                "\"temp_raw\": %d, \"vib_raw\": %d, "
                "\"z_slope\": %.2f, \"t_slope\": %.2f, \"alert_lvl\": %d}",
                msg.raw_x, msg.raw_y, msg.raw_z,
                msg.raw_temp, msg.raw_vib,
                msg.slope_z, msg.slope_temp, msg.alert_level);

            sendto(sockfd, json_payload, strlen(json_payload), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
        }
    }
    return EXIT_SUCCESS;
}
