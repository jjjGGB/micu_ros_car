idf_component_register(
    SRCS "motor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos rotary_encoder pid_ctrl proto_define data_proto 
)
