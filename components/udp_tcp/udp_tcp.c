#include "udp_tcp.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>     // IWYU pragma: export
#include <freertos/event_groups.h> // IWYU pragma: export
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cJSON.h>
#include <esp_check.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_task_wdt.h>

#include "UART.h"
#include "utilities.h"
#include "mdns.h"

#ifndef MDNS_HOSTNAME
#define MDNS_HOSTNAME "smart-car"
#endif // !MDNS_HOSTNAME

#ifndef MDNS_INSTANCE_NAME
#define MDNS_INSTANCE_NAME "ESP32"
#endif // !MDNS_INSTANCE_NAME

#ifndef LISTEN_PORT
#define LISTEN_PORT (8888)
#endif // !LISTEN_PORT

QueueHandle_t s_udp_addr_queue;

/**
 * @brief Parse a JSON string and extra message to addr
 *      
 * 
 * @param[in]   str Raw JSON string
 * @param[out]  addr Address to be written
 * @return int 
 *              - 2 Successfully parse and write ip and port to addr
 *              - 1 Successfully parse and write port to addr(no key named "ip" in JSON string)
 *              - 0 Parse failed
 * 
 * @note        This function don't check validity of arguments
 *              The key "ip" is not necessary, so addr->sin_addr needs to be set when return 1
 */
static int address_parse(const char *str, struct sockaddr_in *addr) {
    int ret = 0;
    cJSON *root = cJSON_Parse(str);
    cJSON *ip;
    cJSON *port;

    addr->sin_family = AF_INET;
    if (NULL == root) {
        ESP_LOGW(TAG, "Not a JSON string");
    } else if (NULL == (port = cJSON_GetObjectItem(root, "port"))) {
        ESP_LOGW(TAG, "No key named \"port\" in JSON");
    } else if (!cJSON_IsNumber(port)) {
        ESP_LOGW(TAG, "The value of \"port\" is not number");
    } else if (NULL == (ip = cJSON_GetObjectItem(root, "ip"))) {
        ret = 1;
        addr->sin_port = htons(port->valueint);
        ESP_LOGI(TAG, "Write port to addr");
    } else if (!cJSON_IsString(ip)) {
        ESP_LOGW(TAG, "The value of \"ip\" is not string");
    } else if (inet_pton(AF_INET, ip->valuestring, &addr->sin_addr) == 0) {
        ESP_LOGW(TAG, "The value of \"ip\" is not a valid ip string");
    } else {
        ret = 2;
        addr->sin_port = htons(port->valueint);
        ESP_LOGI(TAG, "Write ip and port to addr");
    }

    cJSON_Delete(root);
    return ret;
}

static void lidar_udp_forward_task(void *pvParameters) {
    enum { tick_for_delay = pdMS_TO_TICKS(1) > 1 ? pdMS_TO_TICKS(1) : 1 };
    struct sockaddr_in to;
    char ip_str[INET_ADDRSTRLEN];
    uint8_t *buffer[256];
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);//创建UDP socket
    xQueueReceive(s_udp_addr_queue, &to, portMAX_DELAY);//从队列等待第一个目标地址
    inet_ntop(AF_INET, &to.sin_addr, ip_str, sizeof(to));
    ESP_LOGI(TAG, "Got address %s:%u", ip_str, ntohs(to.sin_port));

    while (true) {
        if (xQueueReceive(s_udp_addr_queue, &to, 0)) {//检查是否有新的目标地址更新
            inet_ntop(AF_INET, &to.sin_addr, ip_str, sizeof(to));
            ESP_LOGI(TAG, "Got address %s:%u", ip_str, ntohs(to.sin_port));
        }
        int len = uart_read_bytes(LIDAR_UART_NUM, buffer, sizeof(buffer), 0);
        if (len > 0) {//从UART读取数据通过UDP发送出去
            sendto(sock, buffer, len, 0, (struct sockaddr *)&to, sizeof(to));
        }
        vTaskDelay(tick_for_delay);
    }

    vTaskDelete(NULL);
}

static void tcp_client_handler(void *pvParameters) {
    int sock = (uintptr_t)pvParameters;
    char buffer[128];
    struct sockaddr_in to;
    struct sockaddr from;
    socklen_t fromlen;
    //接收客户端发送的JSON配置
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, &from, &fromlen);
    if (len > 0) {
        buffer[len] = 0;
        int err = address_parse(buffer, &to);//解析JSON获取目标地址信息
        if (1 == err && sizeof(struct sockaddr_in) == fromlen) {//只解析到端口（没有IP字段）
            to.sin_addr.s_addr = ((struct sockaddr_in *)&from)->sin_addr.s_addr;//使用客户端的IP
            xQueueSend(s_udp_addr_queue, &to, portMAX_DELAY);//将解析的地址信息发送到UDP转发任务的队列
        } else if (2 == err) {
            xQueueSend(s_udp_addr_queue, &to, portMAX_DELAY);
        } else {
            ESP_LOGW(TAG, "Invalid JSON string");
        }
    }
    closesocket(sock);
    vTaskDelete(NULL);
}

static void tcp_server_task(void *pvParameters) {
    //创建TCP socket并绑定到8080端口
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(8080),
        .sin_addr.s_addr = INADDR_ANY,
    };
    int tmp = bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    (void)tmp;
    listen(sock, 5);

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        //循环接受新的客户端连接
        int client_sock = accept(sock, (struct sockaddr *)&client_addr, &addr_len);
        xTaskCreate(tcp_client_handler,
            "tcp_handler",
            4096,
            (void *)(uintptr_t)client_sock,
            2,
            NULL);
    }
}

esp_err_t start_mdns_service(void) 
{
	esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(mdns_hostname_set(MDNS_HOSTNAME), TAG, "");// 设置 hostname
    ESP_RETURN_ON_ERROR(mdns_instance_name_set(MDNS_INSTANCE_NAME), TAG, "");// 设置实例名

    return ESP_OK;
}

void tcp_udp_task(void) 
{

    s_udp_addr_queue = xQueueCreate(5, sizeof(struct sockaddr_in));
    ESP_ERROR_CHECK(start_mdns_service());

    xTaskCreate(tcp_server_task,
        "tcp_server",
        4096,
        NULL,
        3,
        NULL);

    xTaskCreate(lidar_udp_forward_task,
        "lidar_udp_forward_task",
        4096,
        NULL,
        4,
        NULL);
}







