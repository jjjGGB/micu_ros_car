#ifndef LIDAR_H_
#define LIDAR_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "esp_log.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "driver/uart.h"
#include "UART.h"

#define frame_head_1                0xAA
#define frame_head_2                0x55
#define lidar_POINT_MAX  		    360  // 雷达点数，一圈360个点
#define lidar_POINT_PER_PACK        40   // 每包协议中的点数

extern SemaphoreHandle_t lidar_sem;

// 单个点云数据
typedef struct {
    uint16_t distance;   // 距离 (mm)
    float angle;         // 角度 (度)
} LidarPoint_t;

// LIDAR 原始数据包
typedef struct {
    uint8_t packet_header[2]; // 帧头 (0x55AA)
    uint8_t CT;               // 包类型
    uint8_t LSN;              // 采样点数量
    uint16_t FSA;             // 起始角
    uint16_t LSA;             // 结束角
    uint16_t CS;              // 校验码
    LidarPoint_t points[lidar_POINT_PER_PACK];  // 采样点云数据 
} LidarPackage_t;

// 应用发布的雷达数据 
typedef struct __attribute__((packed))
{
    uint16_t size;
    LidarPoint_t points[lidar_POINT_MAX];
} LidarPubData_t;

// 解析状态机
typedef enum {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    WAIT_CT,
    WAIT_LSN,
    WAIT_FSA_L,
    WAIT_FSA_H,
    WAIT_LSA_L,
    WAIT_LSA_H,
    WAIT_CS_L,
    WAIT_CS_H,
    WAIT_DATA
} LidarParseState_t;

void  lidar_task(void);
uint16_t Lidar_Get_Distance(uint16_t point);
uint16_t Lidar_Get_Size(void);

#endif /* COMPONENTS_SENSOR_HMC5883L_HMC5883L_H_ */

