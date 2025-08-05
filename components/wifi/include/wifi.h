#ifndef __MWIFI_H__
#define __MWIFI_H__

#include "esp_wifi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"

#include "lwip/ip4_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "nvs.h"

#define WIF_MAX_AP_CONNECTION 2
#define DEFAULT_WIFI_SSID           "Xiaomi_95EE"
#define DEFAULT_WIFI_PASSWORD       "jjt707127ok"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
#define DEFAULT_WIFI_AP_PSWD "87654321"
#define DEFAULT_WIFI_AP_SSID_PREFIX "micu_ros"

typedef enum
{
    WIFI_STATUS_STA_DISCONECTED = 0,
    WIFI_STATUS_STA_CONNECTED = 1,
    WIFI_STATUS_AP_READY,
} wifi_status_t;

bool set_wifi_config(wifi_config_t *wifi_config);

/**
 * @brief 连接到一个热点
 *
 * @param ssid
 * @param pswd
 * @return true
 * @return false
 */
bool wifi_set_sta();


/**
 * @brief wifi初始化
 *
 * @return true
 * @return false
 */
bool wifi_init(void);

/**
 * @brief Get the wifi ip address
 *
 * @param *ip_address
 * @return wifi_status_t
 */
wifi_status_t get_wifi_ip(char *ip_address);
esp_err_t start_mdns_service(const char *hostname, const char *instance_name);
#endif