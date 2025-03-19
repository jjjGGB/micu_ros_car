idf_component_register(
    SRCS "sysn.c"
    INCLUDE_DIRS "." "include" 
    REQUIRES  esp_timer micro_ros_espidf_component imu_publisher
)
