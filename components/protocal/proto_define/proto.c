#include "proto.h"

static protocol_package_t protocol_package_;
static proto_data_motor_encoder_t *proto_motor_encoder_data_;
static proto_data_imu_t *proto_imu_data_;

/**********************************更新PROTOCOL配置的函数************************************/
static update_protocol_config_fun_t update_protocol_config_fun_;

bool proto_register_update_potocol_config_fun(
    update_protocol_config_fun_t *update_protocol_config_fun)
{
  update_protocol_config_fun_ = update_protocol_config_fun;
  return true;
}

void proto_update_potocol_mode_config(
    proto_data_proto_mode_config_t *protocol_config)
{
  update_protocol_config_fun_(protocol_config);
}
/**********************************更新PROTOCOL配置的函数************************************/

/**********************************更新WIFI配置的函数************************************/
static update_wifi_config_fun_t update_wifi_config_fun_;

bool proto_register_update_wifi_config_fun(
    update_wifi_config_fun_t *update_wifi_config_fun)
{
  update_wifi_config_fun_ = update_wifi_config_fun;
  return true;
}

void proto_update_wifi_config(proto_data_wifi_config_t *wifi_config)
{
  update_wifi_config_fun_(wifi_config);
}
/**********************************更新WIFI配置的函数************************************/

/**********************************更新PID配置的函数************************************/

static update_pid_params_fun_t update_pid_params_fun_;// 保存函数指针
bool proto_register_update_pid_fun(
    update_pid_params_fun_t *update_pid_params_fun)
{
  update_pid_params_fun_ = update_pid_params_fun;
  return true;
}
/**********************************更新PID配置的函数************************************/

/**********************************更新电机速度的函数************************************/
static update_speed_params_fun_t update_speed_params_fun_;


bool proto_register_update_motor_speed_fun(// 注册回调函数
    update_speed_params_fun_t *update_motor_speed_ctrl_fun)//函数指针
{
  update_speed_params_fun_ = update_motor_speed_ctrl_fun;// 将传入的函数指针保存到全局变量
  return true;
}

//当上位机发送新的速度控制命令时，通过通信协议接收到数据，然后调用回调函数
void proto_update_motor_speed(proto_motor_speed_ctrl_data_t *motor_speed)
{
  update_speed_params_fun_(motor_speed);
}
/**********************************更新电机速度的函数************************************/

/**********************************设置传感器数据的函数************************************/

void proto_set_motor_encoder_data(
    proto_data_motor_encoder_t *proto_motor_encoder_data)
{
  proto_motor_encoder_data_ = proto_motor_encoder_data;
}

void proto_set_imu_data(proto_data_imu_t *proto_imu_data)
{
  proto_imu_data_ = proto_imu_data;
}
/**********************************设置传感器数据的函数************************************/

uint16_t proto_deal_frame_data(protocol_package_t *protocol_package)
{
  static uint8_t frame[RX_TX_PACKET_SIZE];
  static uint16_t frame_size = 0;
  frame_size = inverse_escape_frame(protocol_package->data, frame,
                                    protocol_package->size);
  return protocol_package->size;
}

/**
 * @brief 将编码器数据转换成数据帧
 *
 * @param frame_data
 * @return uint16_t
 */
uint16_t get_encoder_data(uint8_t *frame_data)
{
  static uint16_t data_header_len = sizeof(proto_data_header_t);
  static uint16_t data_content_len = sizeof(proto_data_motor_encoder_t);

  proto_data_header_t proto_data_header;
  proto_data_header.data_id = DATA_ENCODER;
  proto_data_header.data_len = data_content_len;
  proto_data_header.data_direction = DATA_TO_MASTER;

  // copy
  memcpy(frame_data, &proto_data_header, data_header_len);
  memcpy(frame_data + data_header_len, proto_motor_encoder_data_,
         data_content_len);

  return data_header_len + data_content_len;
}

uint16_t get_imu_data(uint8_t *frame_data)
{
  static uint16_t data_header_len = sizeof(proto_data_header_t);
  static uint16_t data_content_len = sizeof(proto_data_imu_t);

  proto_data_header_t proto_data_header;
  proto_data_header.data_id = DATA_IMU;
  proto_data_header.data_len = data_content_len;
  proto_data_header.data_direction = DATA_TO_MASTER;

  memcpy(frame_data, &proto_data_header, data_header_len);
  memcpy(frame_data + data_header_len, proto_imu_data_, data_content_len);

  return data_header_len + data_content_len;
}

uint16_t proto_get_upload_frame(protocol_package_t **protocol_package)
{
  // update data
  static uint8_t frame_data[RX_TX_PACKET_SIZE]; // 数据域
  static uint8_t frame[RX_TX_PACKET_SIZE + 20]; // 原始帧数据
  static uint8_t frame_index = 0;               // 帧序列
  static uint16_t crc_result = 0;               // crc16校验结果
  static uint16_t data_frame_len = 0;           // 数据域完整的帧的长度
  static uint16_t frame_len = 0;                // 完整的帧的长度
  static uint8_t data_frame_size = 1;           // 数据帧的个数，电机、IMU各算一个

  /*设置本次的初值*/
  data_frame_len = 1;
  data_frame_size = 0;

  /*添加编码器数据帧*/
  data_frame_size++;
  data_frame_len += get_encoder_data(frame_data + data_frame_len);

  // data_frame_size++;
  // data_frame_len += get_encoder_data(frame_data + data_frame_len);

  /*添加IMU数据帧*/
  // data_frame_size++;
  // data_frame_len += get_imu_data(frame_data + data_frame_len);

  /*设置数据帧的数量*/
  frame_data[0] = data_frame_size;

  // 对数据域进行CRC16校验，生成校验码
  crc_result = crc16(frame_data, data_frame_len);

  frame_len = 0;
  frame[frame_len++] = FIRST_CODE;
  frame[frame_len++] = frame_index++;
  frame[frame_len++] = DATA_TARGET_ADDR_PC;
  memcpy(frame + frame_len, frame_data, data_frame_len);
  frame_len += data_frame_len;
  // 小端模式: 高位在前(低内存)，低位在后(高内存)
  frame[frame_len++] = crc_result >> 8;
  frame[frame_len++] = crc_result;
  frame[frame_len++] = END_CODE;
  // 对数据帧进行转义
  protocol_package_.size =
      escape_frame(frame, protocol_package_.data, frame_len);
  *protocol_package = &protocol_package_;
  return (*protocol_package)->size;
}
