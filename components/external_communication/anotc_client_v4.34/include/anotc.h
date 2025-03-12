#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


#include "UART.h"

#include "esp_log.h"
#include "UDP_TCP.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_proto.h"
#include "motor.h"
#include "proto_define.h"

extern float target_speeds[MAX_MOTOR_NUM];
extern proto_data_imu_t proto_imu_data;

void anotc_send(void * pvParameters);
void ANO_DT_Send_voltage(float voltage, float electricity);
void ANO_DT_Send_Sensor_calibration_feedback(uint8_t MSG_ID,uint8_t MSG_DATA);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint32_t alt, uint8_t FLY_ENABLEl, uint8_t armed);
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);

void anotc_task(void);
void anotc_data_decode (char *data);
void ANO_DT_Send_USER_DATA(char command, uint16_t data);



#ifdef __cplusplus
}
#endif

