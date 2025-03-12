#include "motor.h"


static const char *TAG = "motor";
#define PRINT(window, fmt, args...) printf("{"#window"}"fmt"\n", ##args)
#ifdef DRIVER_USE_TB6612
#define UPDATE_OUTPUT(motor_id, output)                                     \
    if (output > 0)                                                        \
    {                                                                       \
        gpio_set_level(motor_config_[motor_id].io_positive, 1);             \
        gpio_set_level(motor_config_[motor_id].io_negative, 0);             \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id],     \
                      (int)output);                                         \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id]); \
    }                                                                       \
    else                                                                    \
    {                                                                       \
        gpio_set_level(motor_config_[motor_id].io_positive, 0);             \
        gpio_set_level(motor_config_[motor_id].io_negative, 1);             \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id],     \
                      -1 * (int)output);                                    \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id]); \
    }

#endif

// 10毫秒目标脉冲数
// 10 ms target pulse number
static float speed_count[MAX_MOTOR_NUM] = {0};

// 通过编码器计算得到电机速度，单位:m/s
// The motor speed is calculated by the encoder, unit :m/s
float current_speeds[MAX_MOTOR_NUM] = {0};

// PID参数结构体
// PID parameter structure
pid_ctrl_parameter_t pid_runtime_param = {0};

// PID电机控制器
// PID motor controller
pid_ctrl_block_handle_t pid_motor[MAX_MOTOR_NUM];


// PID计算后输出的速度值
// PID Output speed value after calculation
static float new_pid_output[MAX_MOTOR_NUM] = {0};
static float pid_target[MAX_MOTOR_NUM] = {0};


/*configs*/
static uint8_t motor_num_ = 0;
pid_ctrl_config_t *pid_config_ = 0;
motor_config_t *motor_config_ = 0;
int16_t target_speeds[MAX_MOTOR_NUM] = {50, 300};    // 电机目标速度，单位mm/s 0-左轮  1-右轮，方向反了，正为反转

static rotary_encoder_t *rotary_encoder_[MAX_MOTOR_NUM]; // 编码器配置
static uint8_t ledc_channel_map[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1,
                                     LEDC_CHANNEL_2, LEDC_CHANNEL_3};


bool set_motor_config(uint8_t motor_num, motor_config_t *motor_configs,
                      pid_ctrl_config_t *pid_configs)
{
    motor_num_ = motor_num;
    pid_config_ = pid_configs;
    motor_config_ = motor_configs;
    return true;
}

uint8_t update_motor_pid_param(proto_data_pid_config_t *proto_pid_data)//回调函数
{

    pid_update_parameters(pid_motor[0],&proto_pid_data);
    printf("PID参数更新成功!\n");
    return true;
}

/**
 * @brief 初始化电机，编码器&PWM
 *
 * @return true
 * @return false
 */
bool motor_init(void)
{
    // get_config && init
    uint8_t i;
    gpio_config_t io_conf;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    

    for (i = 0; i < motor_num_; i++)
    {
        // 编码器初始化
        rotary_encoder_[i] =
        create_rotary_encoder(i, motor_config_[i].io_encoder_positive,
                                  motor_config_[i].io_encoder_negative);
        // pid 配置初始化
        pid_new_control_block(pid_config_ + i, pid_motor + i);
    }


    for (i = 0; i < motor_num_; i++)
    {
        // 方向控制IO初始化
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << motor_config_[i].io_positive) |
                               (1ULL << motor_config_[i].io_negative);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);
        gpio_set_level(motor_config_[i].io_positive, 0);
        gpio_set_level(motor_config_[i].io_negative, 0);

        // PWM初始化
        ledc_channel_config_t ledc_channel = {
            .channel = ledc_channel_map[i],
            .duty = 0,
            .gpio_num = motor_config_[i].io_pwm,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    Motor_Set_Speed(0.5, 0.5);
    proto_register_update_pid_fun(update_motor_pid_param);
    // proto_set_motor_encoder_data(&proto_motor_encoder_data_);
    //proto_register_update_motor_speed_fun(update_motor_speed_fun);//注册更新电机目标速度的函数
    ESP_LOGI(TAG, "init success!");
    return true;
}


static float Motor_Limit_Speed(float speed)
{
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return speed;
}

// PID算法控制电机速度
// PID algorithm controls motor speed
static void Motor_PID_Ctrl(void)
{
    static int last_count[MAX_MOTOR_NUM] = {0};
    static int cur_count[MAX_MOTOR_NUM] = {0};
    static float real_pulse[MAX_MOTOR_NUM] = {0};
    static float new_speed[MAX_MOTOR_NUM] = {0};

    for (int i = 1; i < MAX_MOTOR_NUM; i++)
    {
        cur_count[i] = rotary_encoder_[i]->get_counter_value(rotary_encoder_[i]);
        real_pulse[i] = cur_count[i] - last_count[i];
        last_count[i] = cur_count[i];
        current_speeds[i] = real_pulse[i] * (MOTOR_WHEEL_CIRCLE/MOTOR_ENCODER_CIRCLE/MOTOR_PID_PERIOD);
        PRINT(cnt, " %d, %.2f",cur_count[i],real_pulse[i]);
        
        pid_compute(pid_motor[i], pid_target[i] - real_pulse[i], &new_speed[i]);
        UPDATE_OUTPUT(i, (int)new_speed[i]);
        PRINT(speed, "%.2f, %.2f",new_speed[i],current_speeds[i]);
        PRINT(pid, "%.2f, %.2f",pid_target[i],real_pulse[i]);
        new_pid_output[i] = new_speed[i];
        
    }
}


static void Motor_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Motor_Task with core:%d", xPortGetCoreID());

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        Motor_PID_Ctrl();
        vTaskDelayUntil(&lastWakeTime, MOTOR_PID_PERIOD);
    }

    vTaskDelete(NULL);
}

// 设置电机速度，单位：m/s
// Set motor speed, unit: m/s
void Motor_Set_Speed(float speed_m1, float speed_m2)
{
    static float speed_m[MAX_MOTOR_NUM] = {0};
    speed_m[0] = Motor_Limit_Speed(speed_m1);
    speed_m[1] = Motor_Limit_Speed(speed_m2);
   

    for (int i = 0; i < MAX_MOTOR_NUM; i++)
    {
        // 速度转化成10毫秒编码器目标数量
        // The speed is converted to the number of encoder targets in 10 milliseconds
        speed_count[i] = speed_m[i] / (MOTOR_WHEEL_CIRCLE/MOTOR_ENCODER_CIRCLE/MOTOR_PID_PERIOD);
        pid_target[i] = (float)speed_count[i];
    }
    printf("速度目标值更新成功\n");
}

// 读取当前电机速度值
// Read the current motor speed value
void Motor_Get_Speed(float* speed_current,float* speed_target)
{
    *speed_current = current_speeds[1];//调节右轮
    *speed_target = pid_target[1];
}



// 更新电机PID参数
// Update motor PID parameters
void Motor_Update_PID_Parm(float pid_p, float pid_i, float pid_d)
{
    pid_config_->init_param.kp = pid_p;
    pid_config_->init_param.ki = pid_i;
    pid_config_->init_param.kd = pid_d;
    for (int i = 0; i < MAX_MOTOR_NUM; i++)
    {
        pid_update_parameters(pid_motor[i], &pid_config_->init_param);
    }
    printf("PID参数更新成功!\n");
}

// 读取电机PID参数
// Read motor PID parameters
void Motor_Read_PID_Parm(float* out_p, float* out_i, float* out_d)
{
    *out_p = pid_config_->init_param.kp;
    *out_i = pid_config_->init_param.ki;
    *out_d = pid_config_->init_param.kd;
}

// 初始化编码器电机
// Initialize the encoder motor
void motor_task_init(void)
{
    xTaskCreatePinnedToCore(Motor_Task, "Motor_Task", 10*1024, NULL, 6, NULL, 1);
}

