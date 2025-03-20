#include "lidar.h"

static const char *TAG = "lidar";

LidarPackage_t lidar_frame;
// 解析状态机
static LidarParseState_t parse_state = WAIT_HEADER_1;
static uint8_t data_index = 0;

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
    
    switch (parse_state) {
        case WAIT_HEADER_1:
            if (byte == frame_head_1) {
                parse_state = WAIT_HEADER_2;
            }
            break;

        case WAIT_HEADER_2:
            if (byte == frame_head_2) {
                parse_state = WAIT_CT;
                data_index = 0;
                scan_flag = 1;  // 开始解析点云数据
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
              ESP_LOGI(TAG, "开始解析点云数据");
              point_buffer[data_index++] = byte;

              if (data_index == 2) //两个字节为一个采样数据
              {
                  static uint8_t i = 0;  //当前点云数据的索引，从0开始
                  static float last_angle = -1;  // 记录上一个点的角度
                  static uint8_t similar_flag = 0;  // 记录是否是相同角度的标志位
                  //距离解析
                  uint16_t raw_distance = (point_buffer[1] << 8) | point_buffer[0];
                  lidar_frame.points[i].distance = raw_distance / 4;

                  //角度一级解析，线性插值，按起始角和结束角之间的差值，平均分配到 LSN 个点上
                  float start_angle = (lidar_frame.FSA >> 1) / 64.0f;
                  float end_angle = (lidar_frame.LSA >> 1) / 64.0f;
                  float mid_angle =((start_angle - end_angle)/(lidar_frame.LSN-1)*i)+start_angle;
                  //角度二级解析，基于距离进行角度校正
                  float angle_correction = calculate_angle_correction(raw_distance);
                  mid_angle += angle_correction;

                  // 去掉相同角度的点
                  if (similar_flag == 1) {
                      similar_flag = 0;
                      return 0;  // 如果是相似点，跳过存储
                  } else if ((int)last_angle == (int)mid_angle) {
                      // 如果当前点角度与上一个点角度相同，则计算距离均值
                      lidar_frame.points[i - 1].distance = (lidar_frame.points[i - 1].distance + lidar_frame.points[i].distance) / 2;
                      similar_flag = 1;
                      return 0;
                  }

                  
                  //解析完成，将数据存入点云数据结构体
                  lidar_frame.points[i].angle = mid_angle;
                  last_angle = mid_angle;  // 记录上一个点的角度
                  i++;
                  data_index = 0;

                  //判断是否解析完成
                  if (i >= lidar_frame.LSN) 
                  {
                      ESP_LOGI(TAG, "该数据包解析到: %d 个点云数据", i);
                      for (int i = 0; i < lidar_frame.LSN; i++) 
                      {
                          ESP_LOGI(TAG, "Angle: %.2f°, Distance: %d mm", 
                                  lidar_frame.points[i].angle, 
                                  lidar_frame.points[i].distance);
                      }
                      parse_state = WAIT_HEADER_1;  // 解析完成，回到初始状态
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
    return 0;
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
    vTaskDelay(pdMS_TO_TICKS(1));
	}
		
}



void  lidar_task(void)
{
    xTaskCreate(lidar_data_Task, "lidar_data_Task", 4096 * 8, NULL, 15, NULL);
}



