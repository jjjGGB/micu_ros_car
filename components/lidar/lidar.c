#include "lidar.h"

static const char *TAG = "lidar";

uint8_t rx_raw_buf[100] = {0};
LidarPackage_t lidar_frame;
// 解析状态机
static LidarParseState_t parse_state = WAIT_HEADER_1;
static uint8_t data_index = 0;
LidarPubData_t lidar_pub_data = {0};
uint8_t parse_flag = 0;

// 添加信号量句柄
SemaphoreHandle_t lidar_sem = NULL;

// 计算角度修正值
float calculate_angle_correction(uint16_t distance) {
    if (distance == 0) return 0.0f;
    return atanf(21.8f * (155.3f - distance) / (155.3f * distance));
}


// 逐字节解析 LIDAR 数据
int parse_lidar_byte(uint8_t byte)
 {
    static uint8_t buffer[10];  // 存储帧头信息
    static uint8_t point_buffer[2]; // 存储单个点云数据
    static uint8_t scan_flag = 0;   // 雷达扫描标志位


    if(parse_flag==0)
    {
      switch (parse_state) 
      {
        case WAIT_HEADER_1:
            if (byte == frame_head_1) {
                parse_state = WAIT_HEADER_2;
                //rx_raw_buf[0] = frame_head_1;
            }
            break;

        case WAIT_HEADER_2:
            if (byte == frame_head_2) {
                parse_state = WAIT_CT;
                data_index = 0;
                scan_flag = 1;  // 开始解析点云数据
                //rx_raw_buf[1] = frame_head_2;
            } else {
                parse_state = WAIT_HEADER_1; // 重新等待帧头
            }
            break;

        case WAIT_CT:
            lidar_frame.CT = byte;
            parse_state = WAIT_LSN;
            break;

        case WAIT_LSN:
            lidar_frame.LSN = byte;
            parse_state = WAIT_FSA_L;
            break;

        case WAIT_FSA_L:
            buffer[0] = byte;
            parse_state = WAIT_FSA_H;
            break;

        case WAIT_FSA_H:
            buffer[1] = byte;
            lidar_frame.FSA = (buffer[1] << 8) | buffer[0];
            parse_state = WAIT_LSA_L;
            break;

        case WAIT_LSA_L:
            buffer[2] = byte;
            parse_state = WAIT_LSA_H;
            break;

        case WAIT_LSA_H:
            buffer[3] = byte;
            lidar_frame.LSA = (buffer[3] << 8) | buffer[2];
            parse_state = WAIT_CS_L;
            break;

        case WAIT_CS_L:
            buffer[4] = byte;
            parse_state = WAIT_CS_H;
            break;

        case WAIT_CS_H:
            buffer[5] = byte;
            lidar_frame.CS = (buffer[5] << 8) | buffer[4];
            parse_state = WAIT_DATA;
            data_index = 0; //开始处理点云数据
            break;

        case WAIT_DATA:
            if (scan_flag == 1) 
            {
              //ESP_LOGI(TAG, "开始解析点云数据");
              point_buffer[data_index++] = byte;

              if (data_index == 2) //两个字节为一个采样数据
              {
                  static uint8_t i = 0;  //当前点云数据的索引，从0开始
                  static float last_angle = -1;  // 记录上一个点的角度
                  static uint8_t similar_flag = 0;  // 记录是否是相同角度的标志位
                  //距离解析
                  uint16_t raw_distance = (point_buffer[1] << 8) | point_buffer[0];
                  //角度一级解析，线性插值，按起始角和结束角之间的差值，平均分配到 LSN 个点上
                  float start_angle = (lidar_frame.FSA >> 1) / 64.0f;
                  float end_angle = (lidar_frame.LSA >> 1) / 64.0f;
                  float mid_angle =((start_angle - end_angle)/(lidar_frame.LSN-1)*i)+start_angle;
                  //角度二级解析，基于距离进行角度校正
                  float angle_correction = calculate_angle_correction(raw_distance);
                  mid_angle += angle_correction;
                  
                  //解析完成，将数据存入点云数据结构体
                  lidar_frame.points[i].distance = raw_distance / 4;
                  lidar_frame.points[i].angle = mid_angle;
                  last_angle = mid_angle;  // 记录上一个点的角度
                  i++;
                  data_index = 0;
                  //ESP_LOGI(TAG, "解析到第 %d 个点云数据 成功", i);

                  //判断是否解析完成
                  if (i >= lidar_frame.LSN) 
                  {
                      parse_state = WAIT_HEADER_1;  // 解析完成，回到初始状态
                      parse_flag = 1;
          
                      i = 0;  // 复位点云索引
                      last_angle = -1;  // 复位角度
                      similar_flag=0;
                      scan_flag = 0;  // 停止解析点云数据
                  }
              }
            }
            break;

        default:
            parse_state = WAIT_HEADER_1;
            break;
        }
    }
    return 0;
}


void get_pub_Data(LidarPubData_t* pub_data)
{
    if(xSemaphoreTake(lidar_sem, pdMS_TO_TICKS(100)) == pdTRUE)//如果在100ms内获取到信号量，则获取数据
    {
        memset(pub_data->points, 0, sizeof(LidarPoint_t) * lidar_frame.LSN);  // 清空旧数据
        pub_data->size = lidar_frame.LSN;
        memcpy(pub_data->points, lidar_frame.points, sizeof(LidarPoint_t) * lidar_frame.LSN);
        xSemaphoreGive(lidar_sem);//释放信号量
    // ESP_LOGI(TAG, "发布: %d 个点云数据", pub_data->size);
    // for (int i = 0; i < pub_data->size; i++) 
    // {
    // ESP_LOGI(TAG, "发布点云数据: Angle: %.2f°, Distance: %d mm", 
    //         pub_data->points[i].angle, 
    //         pub_data->points[i].distance);
    // }
    }
}

uint16_t Lidar_Get_Distance(uint16_t point)
{
    if (point < lidar_pub_data.size)
    {
        return lidar_pub_data.points[point].distance;
    }
    return 0;
}

uint16_t Lidar_Get_Size(void)
{
    return lidar_pub_data.size;
}



void lidar_data_Task(void *pvParameter)
{   
  uint16_t rxBytes = 0;
	while (1) 
	{
        rxBytes = Uart_Available(LIDAR_UART_NUM);
        if (rxBytes > 0) {
            for (int i = 0; i < rxBytes; i++) {
                parse_lidar_byte(Uart_Read(LIDAR_UART_NUM));
            }
        }
        if (parse_flag == 1)
        {
            ESP_LOGI(TAG, "解析完成,抽取发布数据");
            get_pub_Data(&lidar_pub_data);
            parse_flag = 0;  
        }
        vTaskDelay(pdMS_TO_TICKS(20));
	}
		
}



void  lidar_task(void)
{
    lidar_sem = xSemaphoreCreateBinary();//创建二值信号量
    xSemaphoreGive(lidar_sem); // 初始状态可用
    xTaskCreatePinnedToCore(lidar_data_Task, "lidar_data_Task", 4096 * 8, NULL, 4, NULL, 1);
}



