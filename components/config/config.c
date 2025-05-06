#include "config.h"


i2c_device_config_t i2c_device_config = {
    .scl_pin = 42,
    .sda_pin = 41,
    .i2c_num = 0,
};

static motor_config_t motor_configs[] = {
    {
        .motor_id = 0,
        .io_pwm = 4,
        .io_positive = 35,//AIN1
        .io_negative = 36,//AIN2
        .io_encoder_positive = 6,
        .io_encoder_negative = 7,
    },
    {
        .motor_id = 1,
        .io_pwm = 5,
        .io_positive = 47,
        .io_negative = 48,
        .io_encoder_positive = 15,
        .io_encoder_negative = 16,
    },
};

static pid_ctrl_config_t pid_config[] = {
    {
        .init_param =
            {
                .kp = 40,
                .ki = 2.5,
                .kd = 0,
                .max_output = PWM_MOTOR_MAX_VALUE,
                .min_output = -PWM_MOTOR_MAX_VALUE,
                .max_integral = 1000,
                .min_integral = -1000,
                .cal_type = PID_CAL_TYPE_POSITIONAL,
            },
    },
    {
        .init_param =
            {
                .kp = 20.0,
                .ki = 0,
                .kd = 3,
                .max_output = 8000,
                .min_output = -8000,
                .max_integral = 1000,
                .min_integral = -1000,
                .cal_type = PID_CAL_TYPE_POSITIONAL,
            },
    },
};


bool config_init()
{

    set_i2c_device_config(&i2c_device_config);
    set_motor_config(DEFAULT_MOTOR_NUM, motor_configs, pid_config);


    printf("1.参数配置初始化完成\n");
    return true;
}

bool hardware_init(void)
{
    char host[22];
    char buf[21]="STAT:WIFI OK";


    if (!wifi_init())
        return false;
    if (!i2c_device_init())
        return false;
    if (!oled_init())
        return false;
    if (!mpu6050_hardware_init())
        return false;
    if (!UART_HARDWARE_init())
        return false;   
    // if(!motor_init())
    //     return false;


    if (get_wifi_ip(host) != WIFI_STATUS_STA_DISCONECTED)
    {  
        oled_ascii8(0,1,buf);
        oled_ascii8(0,2,host);
    }
    printf("2.硬件初始化完成\n");
    return true;
}

bool task_init(void)
{

    mpu6050_task();
    //motor_task_init();
    oled_show_task();
    uart_task();//调试用
    tcp_udp_task();
    //micro_ros_init_task();
    //lidar_task();
    // lidar_pub_task();
    // imu_pub_task();
    esp_log_level_set("MPU6050", ESP_LOG_NONE);  // 不输出IMU模块日志
    printf("3.任务初始化完成\n");
    return true;
}

