idf_component_register(
    SRCS "anotc.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver data_proto motor proto_define UART UDP_TCP
)
