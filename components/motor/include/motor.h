#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "rotary_encoder.h"
#include "pid_ctrl.h"

#include "math.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "proto.h"
#include "proto_define.h"
#include "data_proto.h"

#define LEDC_HIGH_SPEED_MODE  (0)
#define DRIVER_USE_TB6612


#define MOTOR_ENCODER_CIRCLE            (1040)// 电机转动一圈产生的脉冲数量：13*20*4
#define MOTOR_WHEEL_CIRCLE              (150.8)// 轮子周长，单位：mm
#define MOTOR_PID_PERIOD                (10)// PID算法计算周期，单位：ms
#define MOTOR_MAX_SPEED                 (1.0)// 设置电机最大速度，单位：m/s。
#define MOTOR_WHEEL_SPACING             (128.6)// 轮子轮距，单位：mm

// PWM电机时钟频率, 10MHz, 1 tick = 0.1us 
// PWM motor clock frequency, 10MHz, 1 tick = 0.1us
#define PWM_MOTOR_TIMER_RESOLUTION_HZ     80000000   // 80MHz

// PWM电机控制频率, 5KHz (与ledc_timer中设置一致)
// PWM motor control frequency, 25KHz
#define PWM_MOTOR_FREQ_HZ                5000

// PWM理论最大值
// PWM Theoretical maximum (8192)
#define PWM_MOTOR_DUTY_TICK_MAX          (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)

// 电机死区过滤 
// Motor dead zone filtering
#define PWM_MOTOR_DEAD_ZONE              (200)

// 电机PWM输入最大值 
// Maximum motor PWM input value
#define PWM_MOTOR_MAX_VALUE              (PWM_MOTOR_DUTY_TICK_MAX-PWM_MOTOR_DEAD_ZONE)


extern RECEIVE_DATA Receive_Data;

typedef struct
{
    uint8_t motor_id;
    uint8_t io_pwm;
    uint8_t io_positive;//IN1
    uint8_t io_negative;//IN2
    uint8_t io_encoder_positive; // A通道
    uint8_t io_encoder_negative; // B通道
} motor_config_t;

//static void update_motor_output(int motor_id, int output);


/**
 * @brief 初始化配置参数
 *
 * @param motor_num_
 * @param motor_configs
 * @param pid_configs
 * @return true
 * @return false
 */
bool set_motor_config(uint8_t motor_num_, motor_config_t *motor_configs, pid_ctrl_config_t *pid_configs);

/**
 * @brief 更新PID参数
 * 
 * @param proto_pid_data 
 * @return true 
 * @return false 
 */
//uint8_t update_motor_pid_param(fishbot_pid_config_t *proto_pid_data);

/**
 * @brief 初始化电机，编码器&PWM
 *
 * @return true
 * @return false
 */
bool motor_init(void);

/**
 * @brief 电机控制任务
 *
 * @param param
 */
void motor_task_init();

// 添加函数声明
uint8_t update_motor_speed_fun(proto_motor_speed_ctrl_data_t *motor_speed_ctrl);
uint8_t update_motor_pid_param(proto_data_pid_config_t *proto_pid_data);

void Motor_Update_PID_Parm(float pid_p, float pid_i, float pid_d);
void Motor_Read_PID_Parm(float* out_p, float* out_i, float* out_d);
void Motor_Get_Speed(float* speed_current,float* speed_target);

void Motor_Set_Speed(float speed_m1, float speed_m2);

#endif // _MOTOR_H_