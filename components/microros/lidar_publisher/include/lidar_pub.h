#ifndef _LIDAR_PUB_H_
#define _LIDAR_PUB_H_


#include <lidar.h>
#include <sysn.h>
#include "UART.h"
#include "freertos/semphr.h" // 互斥锁需要
extern SemaphoreHandle_t lidar_sem;
extern topic_pub lidar_pub_topic;
void timer_lidar_callback(rcl_timer_t *timer, int64_t last_call_time);

void timer_raw_lidar_callback(rcl_timer_t *timer, int64_t last_call_time);
void lidar_pub_task(void);

#endif
