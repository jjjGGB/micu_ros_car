/*
 * UART1.h
 *
 *  Created on: 2022年11月14日
 *      Author: admin
 */

#ifndef _UART_H_
#define _UART_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "proto.h"
#include "motor.h"
#include "ring_buffer.h"

extern float target_speeds[MAX_MOTOR_NUM];    // 电机目标速度，单位mm/s 0-左轮  1-右轮，方向反了，正为反转
extern float current_speeds[MAX_MOTOR_NUM];  // 电机当前速度，单位mm/s
extern proto_data_imu_t proto_imu_data;

typedef struct {
    uint8_t header;    // 0xAA
    char data[64];     // 数据部分
    uint8_t tail;      // 0x55
} __attribute__((packed)) uart_frame_t;  // 18字节

bool UART_HARDWARE_init(void);
void  uart_task(void);
void uart_init(int baud,int UART_NUM,int TXD_PIN, int RXD_PIN,int RX_BUF_SIZE,int TX_BUF_SIZE);

int Uart0_Send_Byte(uint8_t data);
int Uart0_Send_Data(uint8_t* data, uint16_t len);
uint16_t Uart0_Available(void);
uint8_t Uart0_Read(void);
void Uart0_Clean_Buffer(void);

#endif /* COMPONENTS_UART_INCLUDE_UART1_H_ */
