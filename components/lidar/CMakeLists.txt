idf_component_register(
    SRCS "lidar.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver freertos  UART
)
