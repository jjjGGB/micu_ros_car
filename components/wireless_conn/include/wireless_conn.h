#pragma once

#include "esp_err.h"
#include "mpu6050.h"
#include "controller.h"

typedef struct {
    mc_handle_t left;
    mc_handle_t right;
} motor_control_context_t;

typedef struct {
    mpu6050_handle_t sensor;
    mpu6050_raw_acce_value_t acce_cal;
    mpu6050_raw_gyro_value_t gyro_cal;
    mpu6050_raw_acce_value_t curr_acce;
    mpu6050_raw_gyro_value_t curr_gyro;
} mpu6050_context_t;

typedef struct {
    motor_control_context_t mc;
    mpu6050_context_t mpu6050;
    float wheel_space;
    float wheel_perimeter;
} context_pack_t;

esp_err_t start_wireless_conn(context_pack_t *ctx);
esp_err_t stop_wireless_conn(void);