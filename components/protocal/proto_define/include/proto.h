
#ifndef _PROTO_H_
#define _PROTO_H_

#include "freertos/FreeRTOS.h"
#include "proto_utils.h"
#include "string.h"
#include "esp_system.h"
#include "esp_wifi.h"


#define MAX_MOTOR_NUM 2

/**************************************公共定义结构体******************************************/

/**
 * @brief 数据ID定义
 *
 */
typedef enum
{
  DATA_ENCODER = 0x01, // 0x01-左右轮编码器
  DATA_IMU,            // 0x02-IMU传感器
  DATA_SPEED,          // 0x03-速度控制数据
  DATA_PID,            // 0x04-PID设置
  DATA_VER_INFO,       // 0x05-版本信息
  DATA_WIFI_CONFIG,    // 0x06-WIFI配置
  DATA_DEV_CONTROL,    // 0x07-设备控制
  DATA_PROTO_MODE,     // 0x08-通讯模式设置
} __attribute__((packed)) proto_data_id_t;

/**
 * @brief 数据传输方向定义
 *
 */
typedef enum
{
  DATA_TO_MASTER = 0x01, // 0x01,反馈数据（底盘向主控）
  DATA_TO_FBMC,          // 0x02,命令数据（主控向底盘）
} __attribute__((packed)) proto_data_direction_t;

/**
 * @brief 数据帧的目标地址
 *
 */
typedef enum
{
  DATA_TARGET_ADDR_PC = 0x01,   // 0x01 目标地址电脑
  DATA_TARGET_ADDR_FBMC = 0x02, // 0x02 目标地址FishBot运动控制板
} __attribute__((packed)) proto_data_target_addr_t;


/**
 * @brief 数据段头
 *
 */
typedef struct
{
  proto_data_id_t data_id;               // 数据编号ID
  uint16_t data_len;                     // 数据长度 2
  proto_data_direction_t data_direction; // 数据方向
} __attribute__((packed)) proto_data_header_t;

/**************************************电机控制相关结构体******************************************/

/**
 * @brief PID数据结构体
 *
 */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float max_output;
  float min_output;
  float max_integral;
  float min_integral;
} __attribute__((packed)) proto_data_pid_config_t;

/**
 * @brief 电机编码器数据结构体
 *
 */
typedef struct
{
  int32_t motor_encoder[MAX_MOTOR_NUM]; // 电机的编码器数据
} __attribute__((packed)) proto_data_motor_encoder_t;

/**
 * @brief 电机速度控制结构体
 *
 */
typedef struct
{
  int16_t motor_speed[MAX_MOTOR_NUM]; // 单位mm/s
} __attribute__((packed)) proto_motor_speed_ctrl_data_t;

/**************************************IMU数据结构体******************************************/

/**
 * @brief IMU数据结构体
 *
 */
typedef struct
{
  float accel[3]; // 加速度
  float gyro[3];  // 重力加速度
  float euler[3]; // 欧拉角 RPY
} __attribute__((packed)) proto_data_imu_t;

/**************************************WIFI配置数据结构*******************************************/
/**
 * @brief WIFI配置结构体
 *
 */
typedef struct
{
  wifi_mode_t mode;
  char ssid[16];
  char password[24];
} __attribute__((packed)) proto_data_wifi_config_t;

/**************************************设备控制数据结构*******************************************/
/**
 * @brief 设备编号 0
 *
 */
typedef enum
{
  DEVICE_MOTION_CONTROL = 0x01, // 0x01 运动控制板
  DEVICE_LASER_CONTROL,         // 雷达转接控制板
  DEVICE_POWER_CONTROL,         // 电源管理板
} __attribute__((packed)) proto_device_id_t;

/**
 * @brief 对设备的操作
 *
 */
typedef enum
{
  DEIVCE_RESTART = 0x01, // 0x01 重启设备
  DEVICE_SHUTDOWN,       // 使得设备关机
  DEVICE_SLEEP,          // 使设备进入休眠状态
} __attribute__((packed)) proto_device_operation_t;

/**
 * @brief 设备控制结构体
 *
 */
typedef struct
{
  proto_device_id_t id;
  proto_device_operation_t operation;
} __attribute__((packed)) proto_data_device_control_t;

/*************************************数据域*通讯模式配置定义*******************************************/
/**
 * @brief 通讯模式定义
 *
 */
typedef enum
{
  PROTO_MODE_UPDATE = 0x00,
  PROTO_MODE_UART = 0x01,
  PROTO_MODE_WIFI_UDP_CLIENT,
  PROTO_MODE_WIFI_UDP_SERVER,
} __attribute__((packed)) proto_proto_mode_t;

/**
 * @brief 通讯模式配置结构体
 *
 */
typedef struct
{
  proto_proto_mode_t mode;
  uint32_t bautrate;
  uint16_t port;
  char ip[16];
} __attribute__((packed)) proto_data_proto_mode_config_t;



/*************************************主机->本机数据更新钩子函数*******************************************/

/**
 * @brief 定义更新PID数据的钩子
 *
 */
typedef uint8_t (*update_pid_params_fun_t)(proto_data_pid_config_t *pid);

/**
 * @brief 注册更新PID数据的钩子函数
 *
 * @param update_pid_params_fun
 * @return true
 * @return false
 */
bool proto_register_update_pid_fun(
    update_pid_params_fun_t *update_pid_params_fun);

/*************************************主机->本机 通讯协议配置更新相关函数*******************************************/

/**
 * @brief STEP1.定义更新通讯协议的钩子
 *
 */
typedef uint8_t (*update_protocol_config_fun_t)(
    proto_data_proto_mode_config_t *protocol_config);

/**
 * @brief STEP2: 注册更新电通讯协议的钩子函数
 *
 * @param update_protocol_config_fun_t
 * @return true
 * @return false
 */
bool proto_register_update_potocol_config_fun(
    update_protocol_config_fun_t *update_protocol_config_fun);

/**
 * @brief STEP3: 对外提供更通讯协议函数
 *
 * @param protocol_config
 */
void proto_update_potocol_mode_config(proto_data_proto_mode_config_t *protocol_config);

/*************************************主机->本机 通讯协议配置更新相关函数*******************************************/



/*************************************主机->本机 电机速度更新钩子函数*******************************************/

/**
 * @brief STEP1.定义更新电机速度数据的钩子
 *
 */
typedef uint8_t (*update_speed_params_fun_t)(
    proto_motor_speed_ctrl_data_t *motor_speed);

/**
 * @brief STEP2: 注册更新电机速度数据的钩子函数
 *
 * @param update_motor_speed_ctrl_fun
 * @return true
 * @return false
 */
bool proto_register_update_motor_speed_fun(
    update_speed_params_fun_t *update_motor_speed_ctrl_fun);

/**
 * @brief STEP3: 对外提供更新电机目标速度函数
 *
 * @param motor_speed
 */
void proto_update_motor_speed(proto_motor_speed_ctrl_data_t *motor_speed);

/*************************************主机->本机 WIFI配置更新相关函数*******************************************/

/**
 * @brief STEP1.定义更新WIFI数据的钩子
 *
 */
typedef uint8_t (*update_wifi_config_fun_t)(
    proto_data_wifi_config_t *motor_speed);

/**
 * @brief STEP2: 注册更新WIFI的钩子函数
 *
 * @param update_wifi_config_fun
 * @return true
 * @return false
 */
bool proto_register_update_wifi_config_fun(
    update_wifi_config_fun_t *update_wifi_config_fun);

/**
 * @brief STEP3: 对外提供更新WIFI函数
 *
 * @param wifi_config
 */
void proto_update_wifi_config(proto_data_wifi_config_t *wifi_config);
/*************************************主机->本机 WIFI配置更新相关函数*******************************************/



/*************************************本机->主机数据更新相关函数*******************************************/

/**
 * @brief 设置IMU数据
 *
 * @param proto_data_imu_t
 */
void proto_set_imu_data(proto_data_imu_t *proto_imu_data);

/**
 * @brief 设置电机的编码器数据
 *
 * @param proto_motor_encoder_data
 */
void proto_set_motor_encoder_data(
    proto_data_motor_encoder_t *proto_motor_encoder_data);


/*************************************获取和处理一个数据包*******************************************/

/**
 * @brief 解析和获取一帧数据（根据数据调用不同的钩子完成数据的更新到各个模块）
 *
 * @param protocol_package 数据包指针
 * @return uint16_t
 */
uint16_t proto_deal_frame_data(protocol_package_t *protocol_package);

/**
 * @brief 获取一个需要上传的数据帧（包含传感器的信息组合）
 *
 * @param **protocol_package  指向数据包指针的指针
 * @return uint16_t
 */
uint16_t proto_get_upload_frame(protocol_package_t **protocol_package);




#endif // _PROTO_V1_0_0_220621_H_