#ifndef _IMU_PUB_H_
#define _IMU_PUB_H_

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sysn.h>
#include "proto.h"

extern proto_data_imu_t proto_imu_data;
extern topic_pub test_pub_topic;
extern topic_pub imu_pub_topic;

void imu_ros_init(void);
void imu_update_data_task(void *arg);
void imu_pub_task(void);
void timer1_callback(rcl_timer_t *timer, int64_t last_call_time);
void timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time);

#endif
