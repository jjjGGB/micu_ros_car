#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "oled.h"
#include "wifi.h"
#include <stdbool.h>
#include "mpu6050.h"
#include "motor.h"
#include "lidar.h"
#include "UART.h"

#include "imu_pub.h"
#include "sysn.h"
// 默认的电机数量
#define DEFAULT_MOTOR_NUM 2

/**
 * @brief 初始化参数配置文件
 *
 * @return true
 * @return false
 */
bool config_init(void);

/**
 * @brief 初始化硬件，系统检查
 *
 * @return true
 * @return false
 */
bool hardware_init(void);


/**
 * @brief 加载配置，初始化硬件
 *
 * @return true
 * @return false
 */
bool system_init();

/**
 * @brief 各项任务初始化
 *
 * @return true
 * @return false
 */
bool task_init(void);



#endif 
