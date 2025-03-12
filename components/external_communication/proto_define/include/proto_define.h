#ifndef PROTO_DEFINE_H_
#define PROTO_DEFINE_H_

#include "esp_wifi.h"  // 先包含esp_wifi.h
#include "driver/gpio.h"
#include "proto_utils.h"  // 添加这行
#include "proto.h"     // 再包含proto.h

/******************************应用协议重命名***************************************/
// typedef proto_data_wifi_config_t wifi_config_t;  
// typedef proto_data_pid_config_t pid_config_t;
// typedef proto_data_proto_mode_config_t proto_config_t;

#endif // PROTO_DEFINE_H_