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



// 单个点云数据
typedef struct {
    uint16_t distance;   // 距离 (mm)
    float angle;         // 角度 (度)
} LidarPoint_t;

// LIDAR 数据帧结构体
typedef struct {
    uint8_t packet_header[2]; // 帧头 (0x55AA)
    uint8_t CT;               // 包类型
    uint8_t LSN;              // 采样点数量
    uint16_t FSA;             // 起始角
    uint16_t LSA;             // 结束角
    uint16_t CS;              // 校验码
    LidarPoint_t points[40];  // 采样点云数据 (最多 40 个)
} LidarPackage_t;

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

#endif /* COMPONENTS_SENSOR_HMC5883L_HMC5883L_H_ */

