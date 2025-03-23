#include <imu_pub.h>

static const char *TAG = "IMU_PUB";


// 初始化IMU的ROS话题信息
// Initializes the ROS topic information for IMU
void imu_ros_init(void)
{
    imu_pub_topic.msg.Imu.angular_velocity.x = 0;
    imu_pub_topic.msg.Imu.angular_velocity.y = 0;
    imu_pub_topic.msg.Imu.angular_velocity.z = 0;

    imu_pub_topic.msg.Imu.linear_acceleration.x = 0;
    imu_pub_topic.msg.Imu.linear_acceleration.y = 0;
    imu_pub_topic.msg.Imu.linear_acceleration.z = 0;

    imu_pub_topic.msg.Imu.orientation.x = 0;
    imu_pub_topic.msg.Imu.orientation.y = 0;
    imu_pub_topic.msg.Imu.orientation.z = 0;
    imu_pub_topic.msg.Imu.orientation.w = 1;

    char* content_frame_id = "imu_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    // ESP_LOGI(TAG, "imu frame len:%d", len_frame_id_max);
    char* frame_id = malloc(len_frame_id_max);
    if (len_namespace == 0)
    {
        // ROS命名空间为空字符
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        // 拼接命名空间和frame id
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }
    imu_pub_topic.msg.Imu.header.frame_id = micro_ros_string_utilities_set(imu_pub_topic.msg.Imu.header.frame_id, frame_id);
    free(frame_id);
}

// IMU更新数据任务
// IMU update data task
void imu_update_data_task(void *arg)
{

    while (1)
    {
        imu_pub_topic.msg.Imu.angular_velocity.x = proto_imu_data.gyro[0];
        imu_pub_topic.msg.Imu.angular_velocity.y = proto_imu_data.gyro[1];
        imu_pub_topic.msg.Imu.angular_velocity.z = proto_imu_data.gyro[2];

        imu_pub_topic.msg.Imu.linear_acceleration.x = proto_imu_data.accel[0];
        imu_pub_topic.msg.Imu.linear_acceleration.y = proto_imu_data.accel[0];
        imu_pub_topic.msg.Imu.linear_acceleration.z = proto_imu_data.accel[0];
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}


// 定时器回调函数
// Timer callback function
void timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        imu_pub_topic.msg.Imu.header.stamp.sec = time_stamp.tv_sec;
        imu_pub_topic.msg.Imu.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&imu_pub_topic.publisher, &imu_pub_topic.msg.Imu, NULL));
    }
}

// 定时器timer1回调函数
// Timer1 callback function
void timer1_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&test_pub_topic.publisher, &test_pub_topic.msg.Int32, NULL));
        test_pub_topic.msg.Int32.data++;
    }
}



void imu_pub_task(void)
{
    // 初始化IMU
    // Initialize the IMU
    printf("start imu pub!!!\n");
    imu_ros_init();

    // 开启IMU更新数据任务
    // Start microROS tasks
    xTaskCreatePinnedToCore(imu_update_data_task,
                "imu_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}
