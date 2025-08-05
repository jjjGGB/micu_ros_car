
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include <sys/socket.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "controller.h"
#include "proto_data.h"
#include "wireless_conn.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "esp32_i2c_rw.h"
#include "oled.h"
#include "wifi.h"
#include "UART.h"
#include "app_mpu6050.h"
#include "math.h"

i2c_device_config_t i2c_device_config = {
    .scl_pin = 42,
    .sda_pin = 41,
    .i2c_num = 0,
};

// 轮距，单位：mm
#define MOTOR_WHEEL_SPACING        128.6
// 电机转动一圈产生的脉冲数量：13*20*4
#define PLUSE_PER_ROUND            1040
// 轮子周长，单位：mm
#define MOTOR_WHEEL_CIRCLE         150.8

#define USE_TB6612_DRIVER

#define PID_P   1.15
#define PID_I   1.1
#define PID_D   0.01

static context_pack_t ctx = {
    .wheel_space = MOTOR_WHEEL_SPACING,
    .wheel_perimeter = MOTOR_WHEEL_CIRCLE,
};


static void mc_init(motor_control_context_t *ctx) {
    // 创建定时器循环：PID周期100ms，编码器周期100ms
    mc_timer_loop_create(100, 100);

    // 支持DRV8833和TB6612两种驱动
    mc_config_t config = MC_CONFIG_DEFAULT(PID_P, PID_I, PID_D);
    config.encoder_config.round_count = PLUSE_PER_ROUND;

#ifdef USE_TB6612_DRIVER
    ESP_LOGI("main", "Initializing motors with TB6612 driver");

    // ========== 左电机配置 (TB6612模式) ==========
    config.motor_config.pwma_gpio_num = GPIO_NUM_4;    // PWM信号（实际使用的PWM）
    config.motor_config.pwmb_gpio_num = GPIO_NUM_37;    // 虚拟PWM引脚（TB6612不使用，但库需要有效GPIO）
    config.tb6612_config.pwm_gpio = GPIO_NUM_4;        // TB6612实际PWM信号GPIO
    config.tb6612_config.dir1_gpio = GPIO_NUM_35;       // 方向控制1 (IN1)
    config.tb6612_config.dir2_gpio = GPIO_NUM_36;       // 方向控制2 (IN2)
    config.encoder_config.edge_gpio_num = GPIO_NUM_6;  // 编码器A相
    config.encoder_config.level_gpio_num = GPIO_NUM_7; // 编码器B相

    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->left));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->left));

    //========== 右电机配置 (TB6612模式) ==========
    config.motor_config.pwma_gpio_num = GPIO_NUM_5;    // PWM信号（实际使用的PWM）
    config.motor_config.pwmb_gpio_num = GPIO_NUM_38;    // 虚拟PWM引脚（TB6612不使用，但库需要有效GPIO）
    config.tb6612_config.pwm_gpio = GPIO_NUM_5;        // TB6612实际PWM信号GPIO
    config.tb6612_config.dir1_gpio = GPIO_NUM_47;       // 方向控制1 (IN1)
    config.tb6612_config.dir2_gpio = GPIO_NUM_48;      // 方向控制2 (IN2)
    config.encoder_config.edge_gpio_num = GPIO_NUM_15; // 编码器A相
    config.encoder_config.level_gpio_num = GPIO_NUM_16;// 编码器B相
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->right));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->right));

#else
    ESP_LOGI("main", "Initializing motors with DRV8833 driver");

    // ========== 左电机配置 (DRV8833模式) ==========
    config.motor_config.pwma_gpio_num = GPIO_NUM_36;    // PWM A
    config.motor_config.pwmb_gpio_num = GPIO_NUM_35;    // PWM B
    config.encoder_config.edge_gpio_num = GPIO_NUM_19;  // 编码器A相
    config.encoder_config.level_gpio_num = GPIO_NUM_20; // 编码器B相
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->left));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->left));

    // ========== 右电机配置 (DRV8833模式) ==========
    config.motor_config.pwma_gpio_num = GPIO_NUM_34;    // PWM A
    config.motor_config.pwmb_gpio_num = GPIO_NUM_33;    // PWM B
    config.encoder_config.edge_gpio_num = GPIO_NUM_14;  // 编码器A相
    config.encoder_config.level_gpio_num = GPIO_NUM_12; // 编码器B相
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->right));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->right));
#endif

    // 启动控制循环
    mc_timer_loop_start();
    ESP_LOGI("main", "Motor control system started with %s driver", MOTOR_DRIVER_TYPE);
}

bool hardware_init(void)
{
    static char host[22];
    static char buf[21]="STAT:WIFI OK";

    if (!wifi_init())
        return false;
    if (!set_i2c_device_config(&i2c_device_config))
        return false;
    if (!i2c_device_init())
        return false;
    if (!oled_init())
        return false;
    if (!mpu6050_hardware_init(&ctx.mpu6050))
        return false;
    if (!UART_HARDWARE_init())
        return false;   

    if (get_wifi_ip(host) != WIFI_STATUS_STA_DISCONECTED)
    {  
        oled_ascii8(0,1,buf);
        oled_ascii8(0,2,host);
    }
    printf("硬件初始化完成\n");
    return true;
}

void app_main(void)
{
    printf("Hello 米醋!\n");

    while(hardware_init() == false);

    mc_init(&ctx.mc);
    mc_set_expect_rpm(ctx.mc.left,-20);
    mc_set_expect_rpm(ctx.mc.right,20);
    mpu6050_task(&ctx.mpu6050);
    oled_show_task(&ctx);
    uart_task(&ctx);//调试用
    start_wireless_conn(&ctx);

    printf("任务初始化完成\n");

}
