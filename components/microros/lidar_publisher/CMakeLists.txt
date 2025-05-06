idf_component_register(
    SRCS "lidar_pub.c"
    INCLUDE_DIRS "." "include" 
    REQUIRES proto_define esp_timer micro_ros_espidf_component sysn lidar UART
)
