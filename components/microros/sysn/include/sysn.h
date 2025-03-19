#ifndef _SYSN_H_
#define _SYSN_H_

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>


typedef struct {
    rcl_publisher_t publisher;
    rcl_timer_t timer;
    unsigned int timer_timeout;
    union {
        sensor_msgs__msg__Imu Imu;
        nav_msgs__msg__Odometry Odometry;
        geometry_msgs__msg__Twist Twist;
        sensor_msgs__msg__LaserScan LaserScan;
        std_msgs__msg__Int32 Int32;
    } msg;
} topic_pub;

extern topic_pub test_pub_topic;
extern topic_pub imu_pub_topic ;

unsigned long get_millisecond(void);
void sync_time(void);
struct timespec get_timespec(void);
void micro_ros_init_task(void);

#endif
