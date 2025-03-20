idf_component_register(
    SRCS "UART.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver freertos proto_define motor ring_buffer
)
