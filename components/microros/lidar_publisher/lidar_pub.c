#include <lidar_pub.h>

static const char *TAG = "LIDAR_PUB";
#define MAX_ARRAY_SIZE 200

// 互斥锁
SemaphoreHandle_t lidar_buffer_mutex;
// 双缓冲区
static uint8_t lidar_buffer_a[MAX_ARRAY_SIZE];
static uint8_t lidar_buffer_b[MAX_ARRAY_SIZE];
static size_t lidar_buffer_size_a = 0;
static size_t lidar_buffer_size_b = 0;

// 指向当前采集和发布用的buffer
static uint8_t* lidar_write_buffer = lidar_buffer_a;
static size_t* lidar_write_size = &lidar_buffer_size_a;
static uint8_t* lidar_read_buffer = lidar_buffer_b;
static size_t* lidar_read_size = &lidar_buffer_size_b;





//初始化激光雷达的ROS话题信息
//Initializes the ROS topic information for lidar
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
    int index=0;
    int i = 0;
    while (1)
    {
        if(xSemaphoreTake(lidar_sem, pdMS_TO_TICKS(100)) == pdTRUE) 
        {

            for (i = 0; i < lidar_POINT_MAX; i++)
            {
                distance_mm[i] = Lidar_Get_Distance(i);       
            }
           
            for (i = 0; i < lidar_POINT_MAX; i++)
            {
                 // 1. 先反转顺序 (0->359 变成 359->0)
                index = (lidar_POINT_MAX - i) % lidar_POINT_MAX;  // 359,358,...,1,0
                // 2. 平移180度，使0度对准中间
                if (index >= 180) {
                    // 180-359度 映射到 -180到-1度
                    index = (index - 180) % lidar_POINT_MAX;
                } else {
                    // 0-179度 映射到 0到179度
                    index = (index + 180) % lidar_POINT_MAX;
                }
                lidar_pub_topic.msg.LaserScan.ranges.data[i] = (float)(distance_mm[index] / 1000.0);
                ESP_LOGI(TAG, "发布雷达数据：");
                ESP_LOGI(TAG, "Distance: %.3f m", 
                        lidar_pub_topic.msg.LaserScan.ranges.data[index]);

            }
            xSemaphoreGive(lidar_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(NULL);
}



// 定时器timer1回调函数
// Timer1 callback function
void timer_raw_lidar_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // 交换读写缓冲区指针
        xSemaphoreTake(lidar_buffer_mutex, portMAX_DELAY);
        // 交换指针
        uint8_t* tmp_buf = lidar_write_buffer;
        lidar_write_buffer = lidar_read_buffer;
        lidar_read_buffer = tmp_buf;

        size_t* tmp_size = lidar_write_size;
        lidar_write_size = lidar_read_size;
        lidar_read_size = tmp_size;
        xSemaphoreGive(lidar_buffer_mutex);

        // 拷贝到ROS消息
        memcpy(lidar_pub_topic.msg.UInt8MultiArray.data.data, lidar_read_buffer, *lidar_read_size);
        lidar_pub_topic.msg.UInt8MultiArray.data.size = *lidar_read_size;

        // 发布
        RCSOFTCHECK(rcl_publish(&lidar_pub_topic.publisher, &lidar_pub_topic.msg.UInt8MultiArray, NULL));
        ESP_LOGI(TAG, "发布%d个数据", lidar_pub_topic.msg.UInt8MultiArray.data.size);

        // 清空已发布缓冲区
        memset(lidar_read_buffer, 0, MAX_ARRAY_SIZE);
        *lidar_read_size = 0;
    }
}



void lidar_update_raw_data_task(void *arg)
{
    uint8_t temp_data[MAX_ARRAY_SIZE];
    uint16_t rxBytes = 0;
    while (1)
    {
        memset(temp_data, 0, MAX_ARRAY_SIZE);
        rxBytes = uart_read_bytes(LIDAR_UART_NUM, temp_data, MAX_ARRAY_SIZE, pdMS_TO_TICKS(0));

        if(rxBytes > 0)
        {
            if(rxBytes > MAX_ARRAY_SIZE) rxBytes = MAX_ARRAY_SIZE;

            xSemaphoreTake(lidar_buffer_mutex, portMAX_DELAY);
            memcpy(lidar_write_buffer, temp_data, rxBytes);
            *lidar_write_size = rxBytes;
            xSemaphoreGive(lidar_buffer_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}



void lidar_pub_task(void)
{
    //lidar_ros_init();
    // 分配UInt8MultiArray数据缓冲区
    lidar_pub_topic.msg.UInt8MultiArray.data.data = (uint8_t *)malloc(MAX_ARRAY_SIZE * sizeof(uint8_t));
    lidar_pub_topic.msg.UInt8MultiArray.data.size = 0;  // 初始大小为0
    lidar_buffer_mutex = xSemaphoreCreateMutex();//互斥锁
    printf("start lidar pub!!!\n");
    // 开启激光雷达更新数据任务
    // Start lidar tasks
    // xTaskCreatePinnedToCore(lidar_update_data_task,
    //             "lidar_update_data_task",
    //             CONFIG_MICRO_ROS_APP_STACK,
    //             NULL,
    //             CONFIG_MICRO_ROS_APP_TASK_PRIO,
    //             NULL, 1);

    xTaskCreatePinnedToCore(lidar_update_raw_data_task,
            "lidar_update_raw_data_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL, 1);
    
}
