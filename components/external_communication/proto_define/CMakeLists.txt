set(component_srcs)

idf_component_register(SRCS "proto.c" "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       REQUIRES proto_utils esp_wifi )
