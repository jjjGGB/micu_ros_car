/**
 * @file mpu6050.c
 *
 * @author
 * Gabriel Boni Vicari (133192@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 *
 * @copyright 2018 Jeff Rowberg
 *
 * @brief MPU6050 library for ESP32 ESP-IDF.
 */

#include "mpu6050.h"
#include <string.h>
#include "esp_system.h"  
#include "esp_log.h"  
#include "esp_timer.h"
#include "driver/uart.h"


static const char *TAG = "MPU6050";

#define PRINT(window, fmt, args...) printf("{"#window"}"fmt"\n", ##args)

// #define PI (3.14159265358979323846f)
#define GYRO_MEAS_ERROR (PI * (60.0f / 180.0f))
#define GYRO_MEAS_DRIFT (PI * (1.0f / 180.0f))
#define BETA (sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR)
#define ZETA (sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT)

uint8_t mpu6050_device_address;
uint8_t buffer[14];

#define filter_time 3000 //计算偏差的次数
#define Acc_Gain 16384.0//±2，16位ADC，65535/4=16384
#define Gyro_Gain 131.0//±250，65535/500=131
#define gyroCofe 0.98
#define accCofe 0.02
//---------AHRS算法所用参数-------
#define sampleFreq 10.0f                                                  // 采样频率，从50Hz改为10Hz
#define twoKpDef (2.0f * 0.5f)                                             // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f)                                             // 2 * integral gain
volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
volatile float qqq0 = 1.0f, qqq1 = 0.0f, qqq2 = 0.0f, qqq3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
//-------------------------------
float quart[4] = {1.0f, 0.0f, 0.0f, 0.0f};

float now_t, past_t;
float pitch, yaw, roll;
float yaw_buf[10],yaw1_,yaw11;
float pitch1, yaw1, roll1;
float pitch2, yaw2, roll2;
int yaw_filter_time;
int yaw_filter_flag;
int last_update = 0, first_update = 0, now = 0;

float accoffset[3], gyrooffset[3];
int16_t accel_[3], gyro_[3];
int16_t accel_bias_res[3], gyro_bias_res[3];
float delta_t;
float self_test[6] = {0, 0, 0, 0, 0, 0};

int64_t now_time = 0, prev_time = 0;
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float interval;
float angleGyroX, angleGyroY, angleGyroZ;
float angleX, angleY, angleZ;

proto_data_imu_t proto_imu_data;
// static proto_data_imu_t proto_imu_data;
/******************移植解算所需变量******************/
Vector3f gyro_dps;
Vector3f gyro_raw;
Vector3f accel_g;
Vector3f accel_raw;


// IMU初始化
bool mpu6050_init()
{
    mpu6050_device_address = MPU6050_DEVICE;
    mpu6050_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
    mpu6050_set_full_scale_gyro_range(MPU6050_GYRO_FULL_SCALE_RANGE_250);//原先250
    mpu6050_set_full_scale_accel_range(MPU6050_ACCEL_FULL_SCALE_RANGE_2);//原先2
    mpu6050_set_sleep_enabled(0);

    if (mpu6050_test_connection())
    {
        ESP_LOGI(TAG, "init success!");
        return true;
    }
    else
    {
        ESP_LOGE(TAG, "init failed!");
        return false;
    }
}

bool mpu6050_test_connection()
{
    return (mpu6050_get_device_id() == 0x34);
}

const char *mpu6050_get_tag()
{
    return (TAG);
}

uint8_t mpu6050_get_aux_vddio_level()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_YG_OFFS_TC,
        MPU6050_TC_PWR_MODE_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_aux_vddio_level(uint8_t level)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_YG_OFFS_TC,
        MPU6050_TC_PWR_MODE_BIT,
        level);
}

uint8_t mpu6050_get_rate()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SMPLRT_DIV,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_rate(uint8_t rate)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SMPLRT_DIV,
        rate);
}

uint8_t mpu6050_get_external_frame_sync()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_CONFIG,
        MPU6050_CFG_EXT_SYNC_SET_BIT,
        MPU6050_CFG_EXT_SYNC_SET_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_external_frame_sync(uint8_t sync)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_CONFIG,
        MPU6050_CFG_EXT_SYNC_SET_BIT,
        MPU6050_CFG_EXT_SYNC_SET_LENGTH,
        sync);
}

uint8_t mpu6050_get_dlpf_mode()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_CONFIG,
        MPU6050_CFG_DLPF_CFG_BIT,
        MPU6050_CFG_DLPF_CFG_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_dlpf_mode(uint8_t mode)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_CONFIG,
        MPU6050_CFG_DLPF_CFG_BIT,
        MPU6050_CFG_DLPF_CFG_LENGTH,
        mode);
}

uint8_t mpu6050_get_full_scale_gyro_range()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_GYRO_CONFIG,
        MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_full_scale_gyro_range(uint8_t range)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_GYRO_CONFIG,
        MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH,
        range);
}

uint8_t mpu6050_get_accel_x_self_test_factory_trim()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_X,
        &buffer[0]);
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_A,
        &buffer[1]);

    return ((buffer[0] >> 3) | ((buffer[1] >> 4) & 0x03));
}

uint8_t mpu6050_get_accel_y_self_test_factory_trim()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_Y,
        &buffer[0]);
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_A,
        &buffer[1]);

    return ((buffer[0] >> 3) | ((buffer[1] >> 2) & 0x03));
}

uint8_t mpu6050_get_accel_z_self_test_factory_trim()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_Z,
        2,
        buffer);

    return ((buffer[0] >> 3) | (buffer[1] & 0x03));
}

uint8_t mpu6050_get_gyro_x_self_test_factory_trim()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_X,
        buffer);

    return ((buffer[0] & 0x1F));
}

uint8_t mpu6050_get_gyro_y_self_test_factory_trim()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_Y,
        buffer);

    return ((buffer[0] & 0x1F));
}

uint8_t mpu6050_get_gyro_z_self_test_factory_trim()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_SELF_TEST_Z,
        buffer);

    return ((buffer[0] & 0x1F));
}

bool mpu6050_get_accel_x_self_test()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_XA_ST_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_accel_x_self_test(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_XA_ST_BIT,
        enabled);
}

bool mpu6050_get_accel_y_self_test()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_YA_ST_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_accel_y_self_test(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_YA_ST_BIT,
        enabled);
}

bool mpu6050_get_accel_z_self_test()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_ZA_ST_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_accel_z_self_test(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_ZA_ST_BIT,
        enabled);
}

uint8_t mpu6050_get_full_scale_accel_range()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_AFS_SEL_BIT,
        MPU6050_ACONFIG_AFS_SEL_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_full_scale_accel_range(uint8_t range)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,
        range);
}

uint8_t mpu6050_get_dhpf_mode()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_dhpf_mode(uint8_t mode)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_ACCEL_HPF_BIT,
        MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
        mode);
}

uint8_t mpu6050_get_freefall_detection_threshold()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_FF_THR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_freefall_detection_threshold(uint8_t threshold)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_FF_THR,
        threshold);
}

uint8_t mpu6050_get_freefall_detection_duration()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_FF_DUR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_freefall_detection_duration(uint8_t duration)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_FF_DUR,
        duration);
}

uint8_t mpu6050_get_motion_detection_threshold()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_THR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_motion_detection_threshold(uint8_t threshold)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_THR,
        threshold);
}

uint8_t mpu6050_get_motion_detection_duration()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DUR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_motion_detection_duration(uint8_t duration)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DUR,
        duration);
}

uint8_t mpu6050_get_zero_motion_detection_threshold()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_ZRMOT_THR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_zero_motion_detection_threshold(uint8_t threshold)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_ZRMOT_THR,
        threshold);
}

uint8_t mpu6050_get_zero_motion_detection_duration()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_ZRMOT_DUR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_zero_motion_detection_duration(uint8_t duration)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_ZRMOT_DUR,
        duration);
}

bool mpu6050_get_temp_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_TEMP_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_temp_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_TEMP_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_x_gyro_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_XG_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_x_gyro_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_XG_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_y_gyro_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_YG_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_y_gyro_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_YG_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_z_gyro_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_ZG_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_z_gyro_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_ZG_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_accel_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_ACCEL_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_accel_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_ACCEL_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_2_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_SLV2_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_2_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_SLV2_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_1_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_SLV1_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_1_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_SLV1_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_0_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_SLV0_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_0_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_EN,
        MPU6050_SLV0_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_multi_master_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_MULT_MST_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_multi_master_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_MULT_MST_EN_BIT,
        enabled);
}

bool mpu6050_get_wait_for_external_sensor_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_WAIT_FOR_ES_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_wait_for_external_sensor_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_WAIT_FOR_ES_BIT,
        enabled);
}

bool mpu6050_get_slave_3_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_SLV_3_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_3_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_SLV_3_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_read_write_transition_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_I2C_MST_P_NSR_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_read_write_transition_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_I2C_MST_P_NSR_BIT,
        enabled);
}

uint8_t mpu6050_get_master_clock_speed()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_I2C_MST_CLK_BIT,
        MPU6050_I2C_MST_CLK_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_master_clock_speed(uint8_t speed)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_CTRL,
        MPU6050_I2C_MST_CLK_BIT,
        MPU6050_I2C_MST_CLK_LENGTH,
        speed);
}

uint8_t mpu6050_get_slave_address(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_address(uint8_t num, uint8_t address)
{
    if (num > 3)
        return;

    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3,
        address);
}

uint8_t mpu6050_get_slave_register(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_REG + num * 3,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_register(uint8_t num, uint8_t reg)
{
    if (num > 3)
        return;

    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_REG + num * 3,
        reg);
}

bool mpu6050_get_slave_enabled(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_enabled(uint8_t num, bool enabled)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_word_byte_swap(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_BYTE_SW_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_word_byte_swap(uint8_t num, bool enabled)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_BYTE_SW_BIT,
        enabled);
}

bool mpu6050_get_slave_write_mode(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_REG_DIS_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_write_mode(uint8_t num, bool mode)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_REG_DIS_BIT,
        mode);
}

bool mpu6050_get_slave_word_group_offset(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_GRP_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_word_group_offset(uint8_t num, bool enabled)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_GRP_BIT,
        enabled);
}

uint8_t mpu6050_get_slave_data_length(uint8_t num)
{
    if (num > 3)
        return (0);

    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_LEN_BIT,
        MPU6050_I2C_SLV_LEN_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_data_length(uint8_t num, uint8_t length)
{
    if (num > 3)
        return;

    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3,
        MPU6050_I2C_SLV_LEN_BIT,
        MPU6050_I2C_SLV_LEN_LENGTH,
        length);
}

uint8_t mpu6050_get_slave_4_address()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_ADDR,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_4_address(uint8_t address)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_ADDR,
        address);
}

uint8_t mpu6050_get_slave_4_register()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_REG,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_4_register(uint8_t reg)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_REG,
        reg);
}

void mpu6050_set_slave_4_output_byte(uint8_t data)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_DO,
        data);
}

bool mpu6050_get_slave_4_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_4_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_4_interrupt_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_INT_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_4_interrupt_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_INT_EN_BIT,
        enabled);
}

bool mpu6050_get_slave_4_write_mode()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_REG_DIS_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_4_write_mode(bool mode)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_REG_DIS_BIT,
        mode);
}

uint8_t mpu6050_get_slave_4_master_delay()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_MST_DLY_BIT,
        MPU6050_I2C_SLV4_MST_DLY_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_4_master_delay(uint8_t delay)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_CTRL,
        MPU6050_I2C_SLV4_MST_DLY_BIT,
        MPU6050_I2C_SLV4_MST_DLY_LENGTH,
        delay);
}

uint8_t mpu6050_get_slave_4_input_byte()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV4_DI,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_passthrough_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_PASS_THROUGH_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_slave_4_is_done()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_SLV4_DONE_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_lost_arbitration()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_LOST_ARB_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_slave_4_nack()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_SLV4_NACK_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_slave_3_nack()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_SLV3_NACK_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_slave_2_nack()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_SLV2_NACK_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_slave_1_nack()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_SLV1_NACK_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_slave_0_nack()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_STATUS,
        MPU6050_MST_I2C_SLV0_NACK_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_interrupt_mode()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_INT_LEVEL_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_interrupt_mode(bool mode)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_INT_LEVEL_BIT,
        mode);
}

bool mpu6050_get_interrupt_drive()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_INT_OPEN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_interrupt_drive(bool drive)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_INT_OPEN_BIT,
        drive);
}

bool mpu6050_get_interrupt_latch()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_LATCH_INT_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_interrupt_latch(bool latch)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_LATCH_INT_EN_BIT,
        latch);
}

bool mpu6050_get_interrupt_latch_clear()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_INT_RD_CLEAR_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_interrupt_latch_clear(bool clear)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_INT_RD_CLEAR_BIT,
        clear);
}

bool mpu6050_get_fsync_interrupt_level()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_fsync_interrupt_level(bool level)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT,
        level);
}

bool mpu6050_get_fsync_interrupt_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_FSYNC_INT_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_fsync_interrupt_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_FSYNC_INT_EN_BIT,
        enabled);
}

bool mpu6050_get_i2c_bypass_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_I2C_BYPASS_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_i2c_bypass_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_I2C_BYPASS_EN_BIT,
        enabled);
}

bool mpu6050_get_clock_output_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_CLKOUT_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_clock_output_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_PIN_CFG,
        MPU6050_INTCFG_CLKOUT_EN_BIT,
        enabled);
}

uint8_t mpu6050_get_int_enabled()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_enabled(uint8_t enabled)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        enabled);
}

bool mpu6050_get_int_freefall_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_FF_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_freefall_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_FF_BIT,
        enabled);
}

bool mpu6050_get_int_motion_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_MOT_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_motion_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_MOT_BIT,
        enabled);
}

bool mpu6050_get_int_zero_motion_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_ZMOT_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_zero_motion_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_ZMOT_BIT,
        enabled);
}

bool mpu6050_get_int_fifo_buffer_overflow_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_FIFO_OFLOW_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_fifo_buffer_overflow_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_FIFO_OFLOW_BIT,
        enabled);
}

bool mpu6050_get_int_i2c_master_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_I2C_MST_INT_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_i2c_master_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_I2C_MST_INT_BIT,
        enabled);
}

bool mpu6050_get_int_data_ready_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_DATA_RDY_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_data_ready_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_DATA_RDY_BIT,
        enabled);
}

uint8_t mpu6050_get_int_status()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_freefall_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_FF_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_motion_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_MOT_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_zero_motion_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_ZMOT_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_fifo_buffer_overflow_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_FIFO_OFLOW_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_i2c_master_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_I2C_MST_INT_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_data_ready_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_DATA_RDY_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_get_acceleration(mpu6050_acceleration_t *data)
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_XOUT_H,
        6,
        buffer);
    data->accel_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data->accel_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data->accel_z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t mpu6050_get_acceleration_x()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_XOUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

int16_t mpu6050_get_acceleration_y()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_YOUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

int16_t mpu6050_get_acceleration_z()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_ZOUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

int16_t mpu6050_get_temperature()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_TEMP_OUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_get_rotation(mpu6050_rotation_t *data)
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_GYRO_XOUT_H,
        6,
        buffer);
    data->gyro_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data->gyro_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data->gyro_z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t mpu6050_get_rotation_x()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_GYRO_XOUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

int16_t mpu6050_get_rotation_y()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_GYRO_YOUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

int16_t mpu6050_get_rotation_z()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_GYRO_ZOUT_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_get_motion(
    mpu6050_acceleration_t *data_accel,
    mpu6050_rotation_t *data_gyro)
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ACCEL_XOUT_H,
        14,
        buffer);

    data_accel->accel_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data_accel->accel_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data_accel->accel_z = (((int16_t)buffer[4]) << 8) | buffer[5];
    data_gyro->gyro_x = (((int16_t)buffer[8]) << 8) | buffer[9];
    data_gyro->gyro_y = (((int16_t)buffer[10]) << 8) | buffer[11];
    data_gyro->gyro_z = (((int16_t)buffer[12]) << 8) | buffer[13];
}

uint8_t mpu6050_get_external_sensor_byte(int position)
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_EXT_SENS_DATA_00 + position,
        buffer);

    return (buffer[0]);
}

uint16_t mpu6050_get_external_sensor_word(int position)
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_EXT_SENS_DATA_00 + position,
        2,
        buffer);

    return ((((uint16_t)buffer[0]) << 8) | buffer[1]);
}

uint32_t mpu6050_get_external_sensor_dword(int position)
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_EXT_SENS_DATA_00 + position,
        4,
        buffer);

    return (
        (((uint32_t)buffer[0]) << 24) |
        (((uint32_t)buffer[1]) << 16) |
        (((uint16_t)buffer[2]) << 8) |
        buffer[3]);
}

uint8_t mpu6050_get_motion_status()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_x_negative_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_XNEG_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_x_positive_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_XPOS_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_y_negative_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_YNEG_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_y_positive_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_YPOS_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_z_negative_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_ZNEG_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_z_positive_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_ZPOS_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_zero_motion_detected()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_STATUS,
        MPU6050_MOTION_MOT_ZRMOT_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_output_byte(uint8_t num, uint8_t data)
{
    if (num > 3)
        return;

    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_SLV0_DO + num,
        data);
}

bool mpu6050_get_external_shadow_delay_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_DELAY_CTRL,
        MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_external_shadow_delay_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_DELAY_CTRL,
        MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT,
        enabled);
}

bool mpu6050_get_slave_delay_enabled(uint8_t num)
{
    if (num > 4)
        return (0);

    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_DELAY_CTRL,
        num,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_slave_delay_enabled(uint8_t num, bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_I2C_MST_DELAY_CTRL,
        num,
        enabled);
}

void mpu6050_reset_gyroscope_path()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_SIGNAL_PATH_RESET,
        MPU6050_PATHRESET_GYRO_RESET_BIT,
        1);
}

void mpu6050_reset_accelerometer_path()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_SIGNAL_PATH_RESET,
        MPU6050_PATHRESET_ACCEL_RESET_BIT,
        1);
}

void mpu6050_reset_temperature_path()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_SIGNAL_PATH_RESET,
        MPU6050_PATHRESET_TEMP_RESET_BIT,
        1);
}

uint8_t mpu6050_get_accelerometer_power_on_delay()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_CTRL,
        MPU6050_DETECT_ACCEL_DELAY_BIT,
        MPU6050_DETECT_ACCEL_DELAY_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_accelerometer_power_on_delay(uint8_t delay)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_CTRL,
        MPU6050_DETECT_ACCEL_DELAY_BIT,
        MPU6050_DETECT_ACCEL_DELAY_LENGTH,
        delay);
}

uint8_t mpu6050_get_freefall_detection_counter_decrement()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_CTRL,
        MPU6050_DETECT_FF_COUNT_BIT,
        MPU6050_DETECT_FF_COUNT_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_freefall_detection_counter_decrement(uint8_t decrement)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_CTRL,
        MPU6050_DETECT_FF_COUNT_BIT,
        MPU6050_DETECT_FF_COUNT_LENGTH,
        decrement);
}

uint8_t mpu6050_get_motion_detection_counter_decrement()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_CTRL,
        MPU6050_DETECT_MOT_COUNT_BIT,
        MPU6050_DETECT_MOT_COUNT_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_motion_detection_counter_decrement(uint8_t decrement)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_MOT_DETECT_CTRL,
        MPU6050_DETECT_MOT_COUNT_BIT,
        MPU6050_DETECT_MOT_COUNT_LENGTH,
        decrement);
}

bool mpu6050_get_fifo_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_FIFO_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_fifo_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_FIFO_EN_BIT,
        enabled);
}

bool mpu6050_get_i2c_master_mode_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_I2C_MST_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_i2c_master_mode_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_I2C_MST_EN_BIT,
        enabled);
}

void mpu6050_switch_spie_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_I2C_IF_DIS_BIT,
        enabled);
}

void mpu6050_reset_fifo()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_FIFO_RESET_BIT,
        1);
}

void mpu6050_reset_sensors()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_SIG_COND_RESET_BIT,
        1);
}

void mpu6050_reset()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_DEVICE_RESET_BIT,
        1);
}

bool mpu6050_get_sleep_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_SLEEP_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_sleep_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_SLEEP_BIT,
        enabled);
}

bool mpu6050_get_wake_cycle_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_CYCLE_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_wake_cycle_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_CYCLE_BIT,
        enabled);
}

bool mpu6050_get_temp_sensor_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_TEMP_DIS_BIT,
        buffer);

    return (buffer[0] == 0);
}

void mpu6050_set_temp_sensor_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_TEMP_DIS_BIT,
        !enabled);
}

uint8_t mpu6050_get_clock_source()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_CLKSEL_BIT,
        MPU6050_PWR1_CLKSEL_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_clock_source(uint8_t source)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_1,
        MPU6050_PWR1_CLKSEL_BIT,
        MPU6050_PWR1_CLKSEL_LENGTH,
        source);
}

uint8_t mpu6050_get_wake_frequency()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_LP_WAKE_CTRL_BIT,
        MPU6050_PWR2_LP_WAKE_CTRL_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_wake_frequency(uint8_t frequency)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_LP_WAKE_CTRL_BIT,
        MPU6050_PWR2_LP_WAKE_CTRL_LENGTH,
        frequency);
}

bool mpu6050_get_standby_x_accel_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_XA_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_standby_x_accel_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_XA_BIT,
        enabled);
}

bool mpu6050_get_standby_y_accel_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_YA_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_standby_y_accel_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_YA_BIT,
        enabled);
}

bool mpu6050_get_standby_z_accel_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_ZA_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_standby_z_accel_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_ZA_BIT,
        enabled);
}

bool mpu6050_get_standby_x_gyro_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_XG_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_standby_x_gyro_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_XG_BIT,
        enabled);
}

bool mpu6050_get_standby_y_gyro_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_YG_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_standby_y_gyro_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_YG_BIT,
        enabled);
}

bool mpu6050_get_standby_z_gyro_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_ZG_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_standby_z_gyro_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_PWR_MGMT_2,
        MPU6050_PWR2_STBY_ZG_BIT,
        enabled);
}

uint16_t mpu6050_get_fifo_count()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_COUNTH,
        2,
        buffer);

    return ((((uint16_t)buffer[0]) << 8) | buffer[1]);
}

uint8_t mpu6050_get_fifo_byte()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_R_W,
        buffer);

    return (buffer[0]);
}

void mpu6050_get_fifo_bytes(uint8_t *data, uint8_t length)
{
    if (length > 0)
    {
        esp32_i2c_read_bytes(
            mpu6050_device_address,
            MPU6050_REGISTER_FIFO_R_W,
            length,
            data);
    }
    else
        *data = 0;
}

void mpu6050_set_fifo_byte(uint8_t data)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_FIFO_R_W,
        data);
}

uint8_t mpu6050_get_device_id()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_WHO_AM_I,
        MPU6050_WHO_AM_I_BIT,
        MPU6050_WHO_AM_I_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_device_id(uint8_t id)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_WHO_AM_I,
        MPU6050_WHO_AM_I_BIT,
        MPU6050_WHO_AM_I_LENGTH,
        id);
}

uint8_t mpu6050_get_otp_bank_valid()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_XG_OFFS_TC,
        MPU6050_TC_OTP_BNK_VLD_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_otp_bank_valid(int8_t enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_XG_OFFS_TC,
        MPU6050_TC_OTP_BNK_VLD_BIT,
        enabled);
}

int8_t mpu6050_get_x_gyro_offset_tc()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_XG_OFFS_TC,
        MPU6050_TC_OFFSET_BIT,
        MPU6050_TC_OFFSET_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_x_gyro_offset_tc(int8_t offset)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_XG_OFFS_TC,
        MPU6050_TC_OFFSET_BIT,
        MPU6050_TC_OFFSET_LENGTH,
        offset);
}

int8_t mpu6050_get_y_gyro_offset_tc()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_YG_OFFS_TC,
        MPU6050_TC_OFFSET_BIT,
        MPU6050_TC_OFFSET_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_y_gyro_offset_tc(int8_t offset)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_YG_OFFS_TC,
        MPU6050_TC_OFFSET_BIT,
        MPU6050_TC_OFFSET_LENGTH,
        offset);
}

int8_t mpu6050_get_z_gyro_offset_tc()
{
    esp32_i2c_read_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_ZG_OFFS_TC,
        MPU6050_TC_OFFSET_BIT,
        MPU6050_TC_OFFSET_LENGTH,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_z_gyro_offset_tc(int8_t offset)
{
    esp32_i2c_write_bits(
        mpu6050_device_address,
        MPU6050_REGISTER_ZG_OFFS_TC,
        MPU6050_TC_OFFSET_BIT,
        MPU6050_TC_OFFSET_LENGTH,
        offset);
}

int8_t mpu6050_get_x_fine_gain()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_X_FINE_GAIN,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_x_fine_gain(int8_t gain)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_X_FINE_GAIN,
        gain);
}

int8_t mpu6050_get_y_fine_gain()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_Y_FINE_GAIN,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_y_fine_gain(int8_t gain)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_Y_FINE_GAIN,
        gain);
}

int8_t mpu6050_get_z_fine_gain()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_Z_FINE_GAIN,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_z_fine_gain(int8_t gain)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_Z_FINE_GAIN,
        gain);
}

int16_t mpu6050_get_x_accel_offset()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_XA_OFFS_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_set_x_accel_offset(int16_t offset)
{
    esp32_i2c_write_word(
        mpu6050_device_address,
        MPU6050_REGISTER_XA_OFFS_H,
        offset);
}

int16_t mpu6050_get_y_accel_offset()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_YA_OFFS_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_set_y_accel_offset(int16_t offset)
{
    esp32_i2c_write_word(
        mpu6050_device_address,
        MPU6050_REGISTER_YA_OFFS_H,
        offset);
}

int16_t mpu6050_get_z_accel_offset()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ZA_OFFS_H,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_set_z_accel_offset(int16_t offset)
{
    esp32_i2c_write_word(
        mpu6050_device_address,
        MPU6050_REGISTER_ZA_OFFS_H,
        offset);
}

int16_t mpu6050_get_x_gyro_offset()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_XG_OFFS_USRH,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_set_x_gyro_offset(int16_t offset)
{
    esp32_i2c_write_word(
        mpu6050_device_address,
        MPU6050_REGISTER_XG_OFFS_USRH,
        offset);
}

int16_t mpu6050_get_y_gyro_offset()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_YG_OFFS_USRH,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_set_y_gyro_offset(int16_t offset)
{
    esp32_i2c_write_word(
        mpu6050_device_address,
        MPU6050_REGISTER_YG_OFFS_USRH,
        offset);
}

int16_t mpu6050_get_z_gyro_offset()
{
    esp32_i2c_read_bytes(
        mpu6050_device_address,
        MPU6050_REGISTER_ZG_OFFS_USRH,
        2,
        buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}

void mpu6050_set_z_gyro_offset(int16_t offset)
{
    esp32_i2c_write_word(
        mpu6050_device_address,
        MPU6050_REGISTER_ZG_OFFS_USRH,
        offset);
}

bool mpu6050_get_int_pll_ready_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_PLL_RDY_INT_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_pll_ready_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_PLL_RDY_INT_BIT,
        enabled);
}

bool mpu6050_get_int_dmp_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_DMP_INT_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_int_dmp_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_ENABLE,
        MPU6050_INTERRUPT_DMP_INT_BIT,
        enabled);
}

bool mpu6050_get_dmp_int_5_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_INT_STATUS,
        MPU6050_DMPINT_5_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_dmp_int_4_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_INT_STATUS,
        MPU6050_DMPINT_4_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_dmp_int_3_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_INT_STATUS,
        MPU6050_DMPINT_3_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_dmp_int_2_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_INT_STATUS,
        MPU6050_DMPINT_2_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_dmp_int_1_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_INT_STATUS,
        MPU6050_DMPINT_1_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_dmp_int_0_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_INT_STATUS,
        MPU6050_DMPINT_0_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_ppl_ready_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_PLL_RDY_INT_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_int_dmp_status()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_DMP_INT_BIT,
        buffer);

    return (buffer[0]);
}

bool mpu6050_get_dmp_enabled()
{
    esp32_i2c_read_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_DMP_EN_BIT,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_dmp_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_DMP_EN_BIT,
        enabled);
}

void mpu6050_reset_dmp()
{
    esp32_i2c_write_bit(
        mpu6050_device_address,
        MPU6050_REGISTER_USER_CTRL,
        MPU6050_USERCTRL_DMP_RESET_BIT,
        1);
}

uint8_t mpu6050_get_dmp_config_1()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_CFG_1,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_dmp_config_1(uint8_t config)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_CFG_1,
        config);
}

uint8_t mpu6050_get_dmp_config_2()
{
    esp32_i2c_read_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_CFG_2,
        buffer);

    return (buffer[0]);
}

void mpu6050_set_dmp_config_2(uint8_t config)
{
    esp32_i2c_write_byte(
        mpu6050_device_address,
        MPU6050_REGISTER_DMP_CFG_2,
        config);
}

float mpu6050_get_accel_res(uint8_t accel_scale)
{
    float accel_res = 0;

    switch (accel_scale)
    {
    case 0:
        accel_res = 2.0 / 32768.0;
        break;
    case 1:
        accel_res = 4.0 / 32768.0;
        break;
    case 2:
        accel_res = 8.0 / 32768.0;
        break;
    case 3:
        accel_res = 16.0 / 32768.0;
        break;
    }

    return (accel_res);
}

float mpu6050_get_gyro_res(uint8_t gyro_scale)
{
    float gyro_res = 0;

    switch (gyro_scale)
    {
    case 0:
        gyro_res = 250.0 / 32768.0;
        break;
    case 1:
        gyro_res = 500.0 / 32768.0;
        break;
    case 2:
        gyro_res = 1000.0 / 32768.0;
        break;
    case 3:
        gyro_res = 2000.0 / 32768.0;
        break;
    }

    return (gyro_res);
}

void mpu6050_calibrate(float *accel_bias_res, float *gyro_bias_res)
{
    int32_t accel_bias[3] = {0, 0, 0};
    int32_t gyro_bias[3] = {0, 0, 0};
    int32_t accel_bias_reg[3] = {0, 0, 0};
    uint16_t accel_temp[3] = {0, 0, 0};
    uint16_t gyro_temp[3] = {0, 0, 0};
    uint8_t mask_bit[3] = {0, 0, 0};
    uint32_t mask = 1uL;
    uint16_t gyro_sensitivity = 131;
    uint16_t accel_sensitivity = 16384;
    uint8_t tmp_data[12];
    uint16_t packet_count;

    mpu6050_reset();

    vTaskDelay(pdMS_TO_TICKS(100));//等待100ms

    mpu6050_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Configure device for bias calculation:
    mpu6050_set_int_enabled(0);
    mpu6050_set_fifo_enabled(0);
    mpu6050_set_accel_fifo_enabled(0);
    mpu6050_set_z_gyro_fifo_enabled(0);
    mpu6050_set_y_gyro_fifo_enabled(0);
    mpu6050_set_x_gyro_fifo_enabled(0);
    mpu6050_set_temp_fifo_enabled(0);
    mpu6050_set_clock_source(MPU6050_CLOCK_INTERNAL);
    mpu6050_set_multi_master_enabled(0);
    mpu6050_set_fifo_enabled(0);
    mpu6050_set_i2c_master_mode_enabled(0);
    mpu6050_reset_sensors();
    vTaskDelay(15 / portTICK_PERIOD_MS);

    // Configure MPU6050 gyro and accelerometer for bias calculation:
    mpu6050_set_rate(0x00); // Set sample rate to 1 kHz.
    mpu6050_set_dlpf_mode(MPU6050_DLPF_BW_188);
    mpu6050_set_full_scale_accel_range(MPU6050_ACCEL_FULL_SCALE_RANGE_2);
    mpu6050_set_full_scale_gyro_range(MPU6050_GYRO_FULL_SCALE_RANGE_250);

    /**
     * Configure FIFO to capture data for bias calculation.
     */

    // Enable gyroscope and accelerometer sensors for FIFO:
    mpu6050_set_fifo_enabled(1);
    mpu6050_set_accel_fifo_enabled(1);
    mpu6050_set_z_gyro_fifo_enabled(1);
    mpu6050_set_y_gyro_fifo_enabled(1);
    mpu6050_set_x_gyro_fifo_enabled(1);
    vTaskDelay(80 / portTICK_PERIOD_MS); // Accumulate 80 samples in 80 ms.

    // At end of sample accumulation, turn off FIFO sensor read:
    mpu6050_set_fifo_enabled(0);
    mpu6050_set_accel_fifo_enabled(0);
    mpu6050_set_z_gyro_fifo_enabled(0);
    mpu6050_set_y_gyro_fifo_enabled(0);
    mpu6050_set_x_gyro_fifo_enabled(0);
    mpu6050_set_temp_fifo_enabled(0);

    // Sets of full gyro and accelerometer data for averaging:
    // packet_count 代表滤除偏差的次数，可以根据时间大小修改
    // packet_count = mpu6050_get_fifo_count() / 12;
    packet_count = 5;
    // ESP_LOGI(TAG,"packet_count:%d",packet_count);
    for (int i = 0; i < packet_count; i++)
    {
        // Read data for averaging:
        mpu6050_get_fifo_bytes(&tmp_data[0], 6);
        accel_temp[0] = (int16_t)(((int16_t)tmp_data[0] << 8) | tmp_data[1]);
        accel_temp[1] = (int16_t)(((int16_t)tmp_data[2] << 8) | tmp_data[3]);
        accel_temp[2] = (int16_t)(((int16_t)tmp_data[4] << 8) | tmp_data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)tmp_data[6] << 8) | tmp_data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)tmp_data[8] << 8) | tmp_data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)tmp_data[10] << 8) | tmp_data[11]);

        // Sum individual 16-bit biases to get accumulated signed 32-bit biases:
        accel_bias[0] += (int32_t)accel_temp[0];
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
        ESP_LOGI(TAG, "i:%d", i);
    }

    // Normalize sums to get average count biases:
    accel_bias[0] /= (int32_t)packet_count;
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;
    // ESP_LOGI(TAG, "gyro_bias:%d",gyro_bias[0]);
    //  Remove gravity from the z-axis accelerometer bias calculation:
    if (accel_bias[2] > 0L)
        accel_bias[2] -= (int32_t)accel_sensitivity;
    else
        accel_bias[2] += (int32_t)accel_sensitivity;

    /**
     * Construct the gyro biases for push to the hardware gyro bias registers,
     * which are reset to zero upon device startup:
     */

    // Divide by 4 to get 32.9 LSB per deg/s to expected bias input format.
    // Biases are additive, so change sign on calculated average gyro biases.
    tmp_data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    tmp_data[1] = (-gyro_bias[0] / 4) & 0xFF;
    tmp_data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    tmp_data[3] = (-gyro_bias[1] / 4) & 0xFF;
    tmp_data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    tmp_data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers:
    mpu6050_set_x_gyro_offset(((int16_t)tmp_data[0]) << 8 | tmp_data[1]);
    mpu6050_set_y_gyro_offset(((int16_t)tmp_data[2]) << 8 | tmp_data[3]);
    mpu6050_set_z_gyro_offset(((int16_t)tmp_data[4]) << 8 | tmp_data[5]);

    // Construct gyro bias in deg/s for later manual subtraction:
    gyro_bias_res[0] = (float)gyro_bias[0] / (float)gyro_sensitivity;
    gyro_bias_res[1] = (float)gyro_bias[1] / (float)gyro_sensitivity;
    gyro_bias_res[2] = (float)gyro_bias[2] / (float)gyro_sensitivity;

    /**
     * Construct the accelerometer biases for push to the hardware accelerometer
     * bias registers. These registers contain factory trim values which must be
     * added to the calculated accelerometer biases; on boot up these registers
     * will hold non-zero values. In addition, bit 0 of the lower byte must be
     * preserved since it is used for temperature compensation calculations.
     * Accelerometer bias registers expect bias input as 2048 LSB per g, so that
     * the accelerometer biases calculated above must be divided by 8.
     */

    // Read factory accelerometer trim values:
    tmp_data[0] = mpu6050_get_x_accel_offset();
    tmp_data[1] = mpu6050_get_y_accel_offset();
    tmp_data[2] = mpu6050_get_z_accel_offset();

    for (int i = 0; i < 3; i++)
    {
        // If temperature compensation bit is set, record that in mask_bit:
        if (accel_bias_reg[i] & mask)
            mask_bit[i] = 0x01;
    }

    /**
     * Construct total accelerometer bias, including calculated average
     * accelerometer bias from above (Subtract calculated averaged accelerometer
     * bias scaled to 2048 LSB/g (16g full scale).
     */

    accel_bias_reg[0] -= (accel_bias[0] / 8);
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    tmp_data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    tmp_data[1] = (accel_bias_reg[0]) & 0xFF;
    tmp_data[1] = tmp_data[1] | mask_bit[0];
    tmp_data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    tmp_data[3] = (accel_bias_reg[1]) & 0xFF;
    tmp_data[3] = tmp_data[3] | mask_bit[1];
    tmp_data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    tmp_data[5] = (accel_bias_reg[2]) & 0xFF;
    tmp_data[5] = tmp_data[5] | mask_bit[2];

    // Push accelerometer biases to hardware registers:
    mpu6050_set_x_accel_offset(((int16_t)tmp_data[0]) << 8 | tmp_data[1]);
    mpu6050_set_y_accel_offset(((int16_t)tmp_data[2]) << 8 | tmp_data[3]);
    mpu6050_set_z_accel_offset(((int16_t)tmp_data[4]) << 8 | tmp_data[5]);

    // Output scaled accelerometer biases for subtraction in the main program:
    accel_bias_res[0] = (float)accel_bias[0] / (float)accel_sensitivity;
    accel_bias_res[1] = (float)accel_bias[1] / (float)accel_sensitivity;
    accel_bias_res[2] = (float)accel_bias[2] / (float)accel_sensitivity;
}

void mpu6050_self_test(float *destination)
{
    uint8_t self_test[6];
    float factory_trim[6];

    // Configure the accelerometer for self-test:
    mpu6050_set_accel_x_self_test(true);
    mpu6050_set_accel_y_self_test(true);
    mpu6050_set_accel_z_self_test(true);
    mpu6050_set_full_scale_accel_range(MPU6050_ACCEL_FULL_SCALE_RANGE_8);
    mpu6050_set_full_scale_gyro_range(MPU6050_GYRO_FULL_SCALE_RANGE_250);

    self_test[0] = mpu6050_get_accel_x_self_test_factory_trim();
    self_test[1] = mpu6050_get_accel_y_self_test_factory_trim();
    self_test[2] = mpu6050_get_accel_z_self_test_factory_trim();
    self_test[3] = mpu6050_get_gyro_x_self_test_factory_trim();
    self_test[4] = mpu6050_get_gyro_y_self_test_factory_trim();
    self_test[5] = mpu6050_get_gyro_z_self_test_factory_trim();

    // Process results to allow final comparison with factory set values:
    factory_trim[0] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((self_test[0] - 1.0f) / 30.0f)));
    factory_trim[1] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((self_test[1] - 1.0f) / 30.0f)));
    factory_trim[2] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((self_test[2] - 1.0f) / 30.0f)));
    factory_trim[3] = (25.0f * 131.0f) * (pow(1.046f, (self_test[3] - 1.0f)));
    factory_trim[4] = (-25.0f * 131.0f) * (pow(1.046f, (self_test[4] - 1.0f)));
    factory_trim[5] = (25.0f * 131.0f) * (pow(1.046f, (self_test[5] - 1.0f)));
    // Report results as a ratio of "(STR - FT) / FT" (The change from Factory
    // Trim of the Self-Test Response).
    // To get to percent, must multiply by 100 and subtract result from 100.
    for (int i = 0; i < 6; i++)
        destination[i] = 100.0f + 100.0f * (self_test[i] - factory_trim[i]) / factory_trim[i];
}

void mpu6050_madgwick_quaternion_update(
    float accel_x,
    float accel_y,
    float accel_z,
    float gyro_x,
    float gyro_y,
    float gyro_z)
{
    float func_1, func_2, func_3;
    float j_11o24, j_12o23, j_13o22, j_14o21, j_32, j_33;
    float q_dot_1, q_dot_2, q_dot_3, q_dot_4;
    float hat_dot_1, hat_dot_2, hat_dot_3, hat_dot_4;
    float gyro_x_err, gyro_y_err, gyro_z_err;
    float gyro_x_bias, gyro_y_bias, gyro_z_bias;
    float norm;

    float half_qqq1 = 0.5f * quart[0];
    float half_qqq2 = 0.5f * quart[1];
    float half_qqq3 = 0.5f * quart[2];
    float half_q4 = 0.5f * quart[3];
    float double_qqq1 = 2.0f * quart[0];
    float double_qqq2 = 2.0f * quart[1];
    float double_qqq3 = 2.0f * quart[2];
    float double_q4 = 2.0f * quart[3];

    uint64_t now_time = 0, prev_time = 0;

    // Normalise accelerometer measurement:
    norm = invSqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    
    // Handle NaN:
    if (norm == 0.0f)
        return;

    norm = 1.0f / norm;
    accel_x *= norm;
    accel_y *= norm;
    accel_z *= norm;

    // Compute the objective function and Jacobian:
    func_1 = double_qqq2 * quart[3] - double_qqq1 * quart[2] - accel_x;
    func_2 = double_qqq1 * quart[1] + double_qqq3 * quart[3] - accel_y;
    func_3 = 1.0f - double_qqq2 * quart[3] - double_qqq3 * quart[2] - accel_z;
    j_11o24 = double_qqq3;
    j_12o23 = double_q4;
    j_13o22 = double_qqq1;
    j_14o21 = double_qqq2;
    j_32 = 2.0f * j_14o21;
    j_33 = 2.0f * j_11o24;

    // Compute the gradient (matrix multiplication):
    hat_dot_1 = j_14o21 * func_2 - j_11o24 * func_1;
    hat_dot_2 = j_12o23 * func_1 + j_13o22 * func_2 - j_32 * func_3;
    hat_dot_3 = j_12o23 * func_2 - j_33 * func_3 - j_13o22 * func_1;
    hat_dot_4 = j_14o21 * func_1 + j_11o24 * func_2;

    // Normalize the gradient:
    norm = invSqrt(hat_dot_1 * hat_dot_1 + hat_dot_2 * hat_dot_2 + hat_dot_3 * hat_dot_3 + hat_dot_4 * hat_dot_4);
    hat_dot_1 /= norm;
    hat_dot_2 /= norm;
    hat_dot_3 /= norm;
    hat_dot_4 /= norm;

    // Compute estimated gyroscope biases:
    gyro_x_err = double_qqq1 * hat_dot_2 - double_qqq2 * hat_dot_1 - double_qqq3 * hat_dot_4 + double_q4 * hat_dot_3;
    gyro_y_err = double_qqq1 * hat_dot_3 + double_qqq2 * hat_dot_4 - double_qqq3 * hat_dot_1 - double_q4 * hat_dot_2;
    gyro_z_err = double_qqq1 * hat_dot_4 - double_qqq2 * hat_dot_3 + double_qqq3 * hat_dot_2 - double_q4 * hat_dot_1;

    // Compute and remove gyroscope biases:
    gyro_x_bias += gyro_x_err * interval * ZETA;
    gyro_y_bias += gyro_y_err * interval * ZETA;
    gyro_z_bias += gyro_z_err * interval * ZETA;

    // Compute the quaternion derivative:
    q_dot_1 = -half_qqq2 * gyro_x - half_qqq3 * gyro_y - half_q4 * gyro_z;
    q_dot_2 = half_qqq1 * gyro_x + half_qqq3 * gyro_z - half_q4 * gyro_y;
    q_dot_3 = half_qqq1 * gyro_y - half_qqq2 * gyro_z + half_q4 * gyro_x;
    q_dot_4 = half_qqq1 * gyro_z + half_qqq2 * gyro_y - half_qqq3 * gyro_x;
    //ESP_LOGI(TAG, "1111 qqq0:%.3f,qqq1:%.3f,qqq2:%.3f,qqq3:%.3f",q_dot_1,q_dot_2,q_dot_3,q_dot_4);
    // Compute then integrate estimated quaternion derivative:
    quart[0] += (q_dot_1 - (BETA * hat_dot_1)) * interval;
    quart[1] += (q_dot_2 - (BETA * hat_dot_2)) * interval;
    quart[2] += (q_dot_3 - (BETA * hat_dot_3)) * interval;
    quart[3] += (q_dot_4 - (BETA * hat_dot_4)) * interval;
    // Normalize the quaternion:
    norm = invSqrt(quart[0] * quart[0] + quart[1] * quart[1] + quart[2] * quart[2] + quart[3] * quart[3]);
    norm = 1.0f / norm;
    quart[0] *= norm;
    quart[1] *= norm;
    quart[2] *= norm;
    quart[3] *= norm;

    yaw1= -atan2(2 * qqq1 * qqq2 + 2 * qqq0* qqq3, -2 * qqq2*qqq2 - 2 * qqq3 * qqq3 + 1)*57.3 + 180; // yaw        -pi----pi
    // pitch= -asin(-2 * qqq1 * qqq3 + 2 * qqq0 * qqq2)*57.3; // pitch    -pi/2    --- pi/2 
    // roll= atan2(2 * qqq2 * qqq3 + 2 * qqq0 * qqq1, -2 * qqq1 * qqq1 - 2 * qqq2* qqq2 + 1)*57.3; // roll       -pi-----pi 

	//ESP_LOGI("TAG","qqq0:%.3f,qqq1:%.3f,qqq2:%.3f,qqq3:%.3f",qqq0,qqq1,qqq2,qqq3);
    //ESP_LOGI("TAG","yaw:%.3f,",yaw1);
    yaw_buf[0] = yaw1;
    if(abs(yaw_buf[9]-yaw_buf[0])<=0.0001)
    {
        //ESP_LOGI("TAG","gogogogogo");
        yaw_filter_time++;
        if(yaw_filter_time > 2000)
        {
            
           if(yaw_filter_flag == 0)
           {
            yaw1_ = yaw1; 
            yaw_filter_flag = 1;
            ESP_LOGI(TAG," The filter process is over");
           }
        }
        else
        {
            ESP_LOGI(TAG,"filtering Rest %d",2000-yaw_filter_time);
        }

    }
    for(int i=9;i>0;i--)
    {
        yaw_buf[i] = yaw_buf[i-1];
    }
    //ESP_LOGI("TAG","yaw9:%f   yaw0:%f,  time:%d ",yaw_buf[9],yaw_buf[0],yaw_filter_time);
    yaw11 = yaw1 - yaw1_;
    while(yaw11 < 0 ) yaw11 += 360;
    while(yaw11 > 360 ) yaw11 -=360;
    //ESP_LOGI("TAG","yaw:%.3f,",yaw11);

}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = qqq1 * qqq3 - qqq0 * qqq2;
		halfvy = qqq0 * qqq1 + qqq2 * qqq3;
		halfvz = qqq0 * qqq0 - 0.5f + qqq3 * qqq3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = qqq0;
	qb = qqq1;
	qc = qqq2;
	qqq0 += (-qb * gx - qc * gy - qqq3 * gz);
	qqq1 += (qa * gx + qc * gz - qqq3 * gy);
	qqq2 += (qa * gy - qb * gz + qqq3 * gx);
	qqq3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(qqq0 * qqq0 + qqq1 * qqq1 + qqq2 * qqq2 + qqq3 * qqq3);
	qqq0 *= recipNorm;
	qqq1 *= recipNorm;
	qqq2 *= recipNorm;
	qqq3 *= recipNorm;


	yaw1= -atan2(2 * qqq1 * qqq2 + 2 * qqq0* qqq3, -2 * qqq2*qqq2 - 2 * qqq3 * qqq3 + 1)*57.3 + 180; // yaw        -pi----pi
    // pitch= -asin(-2 * qqq1 * qqq3 + 2 * qqq0 * qqq2)*57.3; // pitch    -pi/2    --- pi/2 
    // roll= atan2(2 * qqq2 * qqq3 + 2 * qqq0 * qqq1, -2 * qqq1 * qqq1 - 2 * qqq2* qqq2 + 1)*57.3; // roll       -pi-----pi 

	//ESP_LOGI("TAG","qqq0:%.3f,qqq1:%.3f,qqq2:%.3f,qqq3:%.3f",qqq0,qqq1,qqq2,qqq3);
    //ESP_LOGI("TAG","yaw:%.3f,",yaw1);
    yaw_buf[0] = yaw1;
    if(abs(yaw_buf[9]-yaw_buf[0])<=0.0001)
    {
        //ESP_LOGI("TAG","gogogogogo");
        yaw_filter_time++;
        if(yaw_filter_time > 2000)
        {
            
           if(yaw_filter_flag == 0)
           {
            yaw1_ = yaw1; 
            yaw_filter_flag = 1;
            ESP_LOGI(TAG," The filter process is over");
           }
        }
        else
        {
            ESP_LOGI(TAG,"filtering Rest %d",2000-yaw_filter_time);
        }

    }
    for(int i=9;i>0;i--)
    {
        yaw_buf[i] = yaw_buf[i-1];
    }
    //ESP_LOGI("TAG","yaw9:%f   yaw0:%f,  time:%d ",yaw_buf[9],yaw_buf[0],yaw_filter_time);
    yaw11 = yaw1 - yaw1_;
    while(yaw11 < 0 ) yaw11 += 360;
    while(yaw11 > 360 ) yaw11 -=360;
    //ESP_LOGI("TAG","yaw:%.3f,",yaw11);
}

// float invSqrt(float x)
// {
//     float halfx = 0.5f * x;
//     float y = x;
//     long i = *(long *)&y;
//     i = 0x5f3759df - (i >> 1);
//     y = *(float *)&i;
//     y = y * (1.5f - (halfx * y * y));
//     return y;
// }

bool mpu6050_hardware_init()
{
    mpu6050_self_test(self_test);
    //ESP_LOGI(TAG, "1111 ax:%f,ay:%f,az:%f,gx:%f,gy:%f,gz:%f",self_test[0],self_test[1],self_test[2],self_test[3],self_test[4],self_test[5]);
    if (self_test[0] < 1.0f && self_test[1] < 1.0f && self_test[2] < 1.0f &&
        self_test[3] < 1.0f && self_test[4] < 1.0f && self_test[5] < 1.0f) {
        mpu6050_reset();
        ESP_LOGI(TAG, "Device being calibrated.");
        mpu6050_init();
        ESP_LOGI(TAG, "Device initialized.");
    }
    else 
        {
            ESP_LOGI(TAG, "Device did not pass self-test.");
            return false;
        }

        ESP_LOGI(TAG, "mpu6050 task init success !.");

    return true;
}

void get_mpu6050_euler_angle(void *param)
{
    while (1)
    {


        now_time = xTaskGetTickCount();
        interval = (float)(now_time - prev_time)/1000.0f;
        //printf("interval:%.2f\n",interval);
        //获取原始数据
        mpu6050_get_motion(accel_,gyro_);  
        
        //陀螺仪 ±2000dps量程下
        //2000 * 2 / 65536 = 0.061035 (65536 = 2^16, 16bit精度)
        gyro_dps.x = gyro_[0] / Gyro_Gain;
        gyro_dps.y = gyro_[1] / Gyro_Gain;
        gyro_dps.z = gyro_[2] / Gyro_Gain;

        // 加速度计 ±16g量程下 65536 / 32 = 2048
        // 2048 LSB/g 灵敏度，直接除以2048是对的
        accel_g.x = accel_[0] / Acc_Gain;
        accel_g.y = accel_[1] / Acc_Gain;
        accel_g.z = accel_[2] / Acc_Gain;

        // 使用新的IMU算法更新姿态
        Axis3f acc = {accel_g.x, accel_g.y, accel_g.z};
        Axis3f gyro = {gyro_dps.x, gyro_dps.y, gyro_dps.z};

        imu_update(acc, gyro, interval);
        //printf("转换后更新数据gyro_:%.2f,%.2f,%.2f\n",gyro.x,gyro.y,gyro.z);
        // 获取欧拉角
        EulerAngles angles = imu_get_euler_angles(gyro);
        vTaskDelay(pdMS_TO_TICKS(10));

        proto_imu_data.euler[0] = angles.roll;
        proto_imu_data.euler[1] = angles.pitch;
        proto_imu_data.euler[2] = angles.yaw;

        proto_imu_data.gyro[0] = gyro.x;
        proto_imu_data.gyro[1] = gyro.y;
        proto_imu_data.gyro[2] = gyro.z;

        proto_imu_data.accel[0] = acc.x;
        proto_imu_data.accel[1] = acc.y;
        proto_imu_data.accel[2] = acc.z;

        //PRINT(imu, "%.3f,%.3f,%.3f",angles.roll,angles.pitch, angles.yaw);
        // printf( "euler angles: roll:%.3f pitch:%.3f yaw:%.3f\n", 
        //         angles.roll,angles.pitch, angles.yaw);
        

        prev_time = now_time;
    }

    
}
void mpu6050_task(void)
{
    xTaskCreatePinnedToCore(get_mpu6050_euler_angle, "get_mpu6050_euler_angle", 8 * 1024, NULL, 10, NULL, 0);
}
