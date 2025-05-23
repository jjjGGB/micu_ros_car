#include <sysn.h>


#include "imu_pub.h"
#include "lidar_pub.h"

static const char *TAG = "MICRO_ROS_INIT";



unsigned long long time_offset = 0;
int handle_num = 3;
topic_pub test_pub_topic = {
    .timer_timeout = 100
};

topic_pub imu_pub_topic = {
    .timer_timeout = 50
};

topic_pub lidar_pub_topic = {
    .timer_timeout = 20
};



// 获取从开机到现在的秒数
// Gets the number of seconds since boot
unsigned long get_millisecond(void)
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

// 计算microROS代理与MCU的时间差
// Calculate the time difference between the microROS agent and the MCU
void sync_time(void)
{
    unsigned long now = get_millisecond();
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

// 获取时间戳
// Get timestamp
struct timespec get_timespec(void)
{
    struct timespec tp = {0};
    // 同步时间 deviation of synchronous time
    unsigned long long now = get_millisecond() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// micro_ros处理任务 
// micro ros processes tasks
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();  // 用于在 ROS 2 节点中分配和释放内存
    rclc_support_t support; // 用于在 ROS 2 上下文中初始化和配置执行器、节点等资源

    // 创建rcl初始化选项
    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // 修改ROS域ID
    // change ros domain id
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // 初始化rmw选项
    // Initialize the rmw options
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // 设置静态代理IP和端口
    // Setup static agent IP and port
    RCCHECK(rmw_uros_options_set_udp_address(ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));

    // 尝试连接代理，连接成功才进入下一步。
    // Try to connect to the agent. If the connection succeeds, go to the next step.
    int state_agent = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Connecting agent: %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected agent: %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // 创建ROS2节点
    // create ROS2 node
    rcl_node_t node;
    // 调用 rclc_node_init_default 函数初始化 ROS 2 节点，传入节点名称、命名空间和支持库
    RCCHECK(rclc_node_init_default(&node, "esp32_publisher", ROS_NAMESPACE, &support));

    // 创建imu发布者
    // create publisher_imu
    RCCHECK(rclc_publisher_init_default(
        &imu_pub_topic.publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_pub"));

    // 创建发布者1
    // create publisher_1
    RCCHECK(rclc_publisher_init_default(
        &test_pub_topic.publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "test_pub"));

    // 创建lidar发布者
    // create publisher_lidar
    // RCCHECK(rclc_publisher_init_default(
    //     &lidar_pub_topic.publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    //     "scan"));

    RCCHECK(rclc_publisher_init_default(
        &lidar_pub_topic.publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "raw_scan"));

    // 创建定时器，设置发布频率
    // create timer. Set the publish frequency to 20HZ

    RCCHECK(rclc_timer_init_default(
        &imu_pub_topic.timer,
        &support,
        RCL_MS_TO_NS(imu_pub_topic.timer_timeout),
        timer_imu_callback));

    RCCHECK(rclc_timer_init_default(
        &test_pub_topic.timer,
        &support,
        RCL_MS_TO_NS(test_pub_topic.timer_timeout),
        timer1_callback));

    // RCCHECK(rclc_timer_init_default(
    //     &lidar_pub_topic.timer,
    //     &support,
    //     RCL_MS_TO_NS(lidar_pub_topic.timer_timeout),
    //     timer_lidar_callback));
    
    RCCHECK(rclc_timer_init_default(
        &lidar_pub_topic.timer,
        &support,
        RCL_MS_TO_NS(lidar_pub_topic.timer_timeout),
        timer_raw_lidar_callback));

    // 创建执行者，其中三个参数为执行者控制的数量，要大于或等于添加到执行者的订阅者和发布者数量。
    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;  // 用于在单个线程中处理多个 ROS 2 资源的回调函数。


    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    
    // 添加发布者的定时器到执行者
    // Adds the publisher_imu's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &imu_pub_topic.timer));
    RCCHECK(rclc_executor_add_timer(&executor, &test_pub_topic.timer));
    RCCHECK(rclc_executor_add_timer(&executor, &lidar_pub_topic.timer));

    sync_time();

    // 循环执行microROS任务
    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // 释放资源
    // free resources
    RCCHECK(rcl_publisher_fini(&imu_pub_topic.publisher, &node));
    RCCHECK(rcl_publisher_fini(&test_pub_topic.publisher, &node));
    RCCHECK(rcl_publisher_fini(&lidar_pub_topic.publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    free(lidar_pub_topic.msg.UInt8MultiArray.data.data);

    vTaskDelete(NULL);
}

void micro_ros_init_task(void)
{

    printf(" Start microROS tasks!!!");
    // 开启microROS任务
    // Start microROS tasks
    xTaskCreatePinnedToCore(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}
