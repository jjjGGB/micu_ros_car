#include <lidar_pub.h>


static const char *TAG = "LIDAR_PUB";



// 初始化激光雷达的ROS话题信息
// Initializes the ROS topic information for lidar
void lidar_ros_init(void)
{
    int i;

    lidar_pub_topic.msg.LaserScan.angle_min = -180*M_PI/180.0;
    lidar_pub_topic.msg.LaserScan.angle_max = 180*M_PI/180.0;

    lidar_pub_topic.msg.LaserScan.angle_increment = 1*M_PI/180.0;
    lidar_pub_topic.msg.LaserScan.range_min = 0.05;
    lidar_pub_topic.msg.LaserScan.range_max = 5.0;

    lidar_pub_topic.msg.LaserScan.ranges.data = (float *)malloc(360 * sizeof(float));
    lidar_pub_topic.msg.LaserScan.ranges.size = 360;
    for (i = 0; i < lidar_pub_topic.msg.LaserScan.ranges.size; i++)
    {
        lidar_pub_topic.msg.LaserScan.ranges.data[i] = 0;
    }
    
    char* content_frame_id = "laser_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    // ESP_LOGI(TAG, "lidar frame len:%d", len_frame_id_max);
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
    lidar_pub_topic.msg.LaserScan.header.frame_id = 
        micro_ros_string_utilities_set(lidar_pub_topic.msg.LaserScan.header.frame_id, frame_id);
    free(frame_id);
}

// 定时器回调函数
// Timer callback function
void timer_lidar_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        lidar_pub_topic.msg.LaserScan.header.stamp.sec = time_stamp.tv_sec;
        lidar_pub_topic.msg.LaserScan.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&lidar_pub_topic.publisher, &lidar_pub_topic.msg.LaserScan, NULL));
    }
}

// 激光雷达更新数据任务
// lidar update data task
void lidar_update_data_task(void *arg)
{
    uint16_t distance_mm[lidar_POINT_MAX] = {0};
    uint16_t size = 0;
    uint16_t index = 0;
    int i = 0;
    while (1)
    {
        index = 0;
        if(xSemaphoreTake(lidar_sem, pdMS_TO_TICKS(100)) == pdTRUE) 
        {
            size = Lidar_Get_Size();
            for (i = 0; i < size; i++)
            {
                distance_mm[i] = Lidar_Get_Distance(i);
                lidar_pub_topic.msg.LaserScan.ranges.data[i] = (float)(distance_mm[i] / 1000.0);
            }
            xSemaphoreGive(lidar_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(NULL);
}

void lidar_pub_task(void)
{
    lidar_ros_init();

    // 开启激光雷达更新数据任务
    // Start lidar tasks
    xTaskCreatePinnedToCore(lidar_update_data_task,
                "lidar_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}