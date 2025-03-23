idf_component_register(
    SRCS "config.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_wifi wifi oled mpu6050 motor esp32_i2c_rw lidar UART  imu_publisher
    sysn lidar_publisher
)
