idf_component_register(
    SRCS "UDP_TCP.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver lidar esp_netif esp_event data_proto   motor proto_define mdns
)
