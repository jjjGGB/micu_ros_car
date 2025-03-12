#ifndef COMPONENTS_EXTERNAL_COMMUNICATION_UDP_TCP_INCLUDE_UDP_TCP_H_
#define COMPONENTS_EXTERNAL_COMMUNICATION_UDP_TCP_INCLUDE_UDP_TCP_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_event.h"
#include <esp_check.h>
#include "data_proto.h"
#include "motor.h"
#include "proto.h"

extern float current_speeds[MAX_MOTOR_NUM];
extern SEND_DATA Send_Data;
extern RECEIVE_DATA Receive_Data;
extern proto_data_imu_t proto_imu_data;

void TCP_write_motion(uint8_t *data,uint8_t len);
void UDP_write_anotc(uint8_t *data,uint8_t len);//UDP发送
void udp_write_lidar(uint8_t *data,uint8_t len);//lidar激光雷达DUP发送

bool UDP_init(int *sock);
esp_err_t UDP_task(int sock);

#endif /* COMPONENTS_EXTERNAL_COMMUNICATION_UDP_TCP_INCLUDE_UDP_TCP_H_ */
