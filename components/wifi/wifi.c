#include "wifi.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define STA_MODE

static const char *TAG = "WIFI";

static proto_data_wifi_config_t *wifi_config_ = NULL;
static wifi_status_t wifi_status_ = WIFI_STATUS_STA_DISCONECTED;
static char wifi_ip_[16];
static uint8_t retry_connect_sta_count_ = 0;
static esp_netif_t *ap_netif = NULL;  



/** 事件回调函数
 * @param arg   用户传递的参数
 * @param event_base    事件类别
 * @param event_id      事件ID
 * @param event_data    事件携带的数据
 * @return 无
*/
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if(event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:      //WIFI以STA模式启动后触发此事件
            esp_wifi_connect();         //启动WIFI连接
            break;

        case WIFI_EVENT_STA_DISCONNECTED:  //WIFI连上路由器后，触发此事件
            {
                if (retry_connect_sta_count_ < EXAMPLE_ESP_MAXIMUM_RETRY) 
                {
                    wifi_status_ = WIFI_STATUS_STA_DISCONECTED;
                    retry_connect_sta_count_++;
                    esp_wifi_connect();
                    ESP_LOGI(TAG, "retry to connect to the STA");
                } else {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                ESP_LOGI(TAG,"connect to the STA fail");

            }
            break;

        case WIFI_EVENT_AP_STACONNECTED://有设备连接
            {
                wifi_event_ap_staconnected_t *event =
                    (wifi_event_ap_staconnected_t *)event_data; //获取连接设备信息信息

                printf("有设备连接\n");
            }
            break;

        case WIFI_EVENT_AP_STADISCONNECTED:
            {
                wifi_event_ap_stadisconnected_t *event =
                    (wifi_event_ap_stadisconnected_t *)event_data;
                printf("有设备断开\n");
            }
            break;

        default:
            break;
        }
    }
    else if(event_base == IP_EVENT)                  //IP相关事件
    {
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:           //只有获取到路由器分配的IP，才认为是连上了路由器
                {
                    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
                    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
                    
                    char ip_str[16];
                    esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
                    sprintf(wifi_ip_, "%s", ip_str);
                    retry_connect_sta_count_=0;
                    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                    wifi_status_ = WIFI_STATUS_STA_CONNECTED; 
                }
                break;
        }
    }
    

}

bool set_wifi_config(wifi_config_t *wifi_config)
{
    wifi_config_ = wifi_config;
    return true;
}


bool wifi_set_ap()
{
    esp_err_t ret = "ESP-OK"; 
    uint8_t mac[6];

    ap_netif = esp_netif_create_default_wifi_ap();  // 保存返回的netif句柄
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap =
            {
                .ssid = DEFAULT_WIFI_AP_SSID_PREFIX,
                .password = DEFAULT_WIFI_AP_PSWD,
                .max_connection = WIF_MAX_AP_CONNECTION,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));

    // sprintf((char *)wifi_config.ap.ssid, "%s_%02X%02X", ssid, mac[0], mac[1]);
    // strcpy((char *)wifi_config.ap.password, pswd);

    // if (strlen(pswd) == 0)
    // {
    //     wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    // }


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             wifi_config.ap.ssid, wifi_config.ap.password);

    esp_netif_ip_info_t ip_info = {//IPV4地址结构体
			.ip.addr = ipaddr_addr("192.168.43.100"),//IPV4地址
			.netmask.addr = ipaddr_addr("255.255.255.0"),//IPV4子掩码
			.gw.addr      = ipaddr_addr("192.168.43.1"),//IPV4网关地址
	};

	if(esp_netif_dhcps_stop(ap_netif) == ESP_OK) printf("停止DHCP 服务器成功\n");//要修改 IP 配置，需要停止 DHCP 服务器
	else printf("停止DHCP 服务器失败\n");

	ret = esp_netif_set_ip_info(ap_netif, &ip_info);//设置IP地址
	if(ret == ESP_OK)
    {
         sprintf(wifi_ip_, "%s", "192.168.43.100");
         printf("IP地址设置成功\nIPV4地址:192.168.43.100 \nIPV4子掩码:255.255.255.0 \nIPV4网关地址:192.168.43.42 \n");
    }
	else printf("IP地址设置失败\n");



	if(esp_netif_dhcps_start(ap_netif) == ESP_OK) printf("重新启动DHCP 服务器成功\n");//重新启动 DHCP 服务器
	else printf("重新启动DHCP 服务器失败\n");

    wifi_status_ = WIFI_STATUS_AP_READY;
    return true;
}


bool wifi_set_sta()
{
    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = DEFAULT_WIFI_SSID,
                .password = DEFAULT_WIFI_PASSWORD,
                .scan_method = WIFI_FAST_SCAN ,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                .threshold.authmode = WIFI_AUTH_OPEN, //加密方式
            },
    };

    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();    //使用默认配置创建STA对象

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();//初始化WIFI配置结构体
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_wifi_set_ps(WIFI_PS_NONE);//禁用Wi-Fi省电模式，确保设备在连接过程中不会进入低功耗模式。

    //注册事件
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    //启动WIFI
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));//设置工作模式为STA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));//设置wifi配置
    ESP_ERROR_CHECK(esp_wifi_start());//启动WIFI

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    //等待连接结果
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        10 * 1000 / portTICK_RATE_MS);//等待最多10秒

    // //处理连接结果
    // if (bits & WIFI_CONNECTED_BIT)
    // {
    //     ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
    //              wifi_config.ap.ssid, wifi_config.ap.password);
    // }
    // else if (bits & WIFI_FAIL_BIT)
    // {
    //     ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
    //              wifi_config.ap.ssid, wifi_config.ap.password);
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "UNEXPECTED EVENT");
    // }

    //注销事件处理函数,删除事件组
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
    if (wifi_status_ != WIFI_STATUS_STA_CONNECTED)
    {
        return true;
    }
    return false;
}


bool wifi_init()
{
    //NVS初始化（WIFI底层驱动有用到NVS，所以这里要初始化）
    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());//用于初始化tcpip协议栈
    ESP_ERROR_CHECK(esp_event_loop_create_default()); //创建一个默认系统事件调度循环，之后可以注册回调函数来处理系统的一些事件
    #ifdef STA_MODE
    wifi_set_sta();
    #else
    wifi_set_ap();
    #endif
    ESP_LOGI(TAG, "wifi init success!");
    return true;
}

wifi_status_t get_wifi_ip(char *ip_address)
{

    if (wifi_status_ == WIFI_STATUS_STA_CONNECTED)
    {
        sprintf(ip_address, "HOST:%s", wifi_ip_);
    }
    else if (wifi_status_ == WIFI_STATUS_STA_DISCONECTED)
    {
        sprintf(ip_address, "HOST:lost_connect");
    }
    else if (wifi_status_ == WIFI_STATUS_AP_READY)
    {
        sprintf(ip_address, "HOST:%s", wifi_ip_);
    }
    return wifi_status_;
}