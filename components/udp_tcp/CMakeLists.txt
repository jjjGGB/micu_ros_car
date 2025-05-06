idf_component_register(
    SRCS "udp_tcp.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver freertos  UART  json mdns
)
