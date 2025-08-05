idf_component_register(
    SRCS "UART.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver freertos  motor ring_buffer wireless_conn
)
