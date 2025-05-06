#ifndef _SYSN_H_
#define _SYSN_H_

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>
#include <uros_network_interfaces.h>


#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/u_int8_multi_array.h>


#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

#define ROS_NAMESPACE      CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID      CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP       CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT     CONFIG_MICRO_ROS_AGENT_PORT

typedef struct {
    rcl_publisher_t publisher;
    rcl_timer_t timer;// 用于在指定的时间间隔内执行回调函数
    unsigned int timer_timeout;
    union {
        sensor_msgs__msg__Imu Imu;
        nav_msgs__msg__Odometry Odometry;
        geometry_msgs__msg__Twist Twist;
        sensor_msgs__msg__LaserScan LaserScan;
        std_msgs__msg__Int32 Int32;
        std_msgs__msg__UInt8MultiArray UInt8MultiArray;
    } msg;
} topic_pub;

extern topic_pub test_pub_topic;
extern topic_pub imu_pub_topic ;
extern topic_pub lidar_pub_topic;

unsigned long get_millisecond(void);
void sync_time(void);
struct timespec get_timespec(void);
void micro_ros_init_task(void);

#endif
