#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "esp_check.h"
#include "cJSON.h"
#include "UART.h"
#include "proto_data.h"
#include "wireless_conn.h"

#define TAG "ros_conn"

#define NEXT_POW2(x) (1 << ( (8*sizeof(x) - __builtin_clz( (x | (x-1)) + 1 )) ))

static int g_socks[] = { -1, -1, -1,-1 };
static TaskHandle_t g_task_handles[4];
static SemaphoreHandle_t g_chassis_send_muetx;
static SemaphoreHandle_t g_chassis_recv_muetx;
static SemaphoreHandle_t g_remote_muetx;
static SemaphoreHandle_t g_lidar_send_muetx;
#define g_conn_sock (g_socks[0])
#define g_chassis_sock (g_socks[1])
#define g_remote_sock (g_socks[2])
#define g_lidar_sock (g_socks[3])

static context_pack_t *g_ctx;
static bool g_ros_ctrl = false;
static struct sockaddr_in g_lidar_target_addr;  // 添加全局目标地址
static bool g_has_lidar_target = false;  // 添加全局标志位
static struct sockaddr_in g_chassis_target_addr;  // 添加全局目标地址


static void ros_send_task(void *pvParameters) {
    int sock = (int)pvParameters;
    uint8_t temp = 6;  
    for (;;) {
        float l_rpm = -mc_get_real_rpm(g_ctx->mc.left);
        float r_rpm = mc_get_real_rpm(g_ctx->mc.right);
        ros_send_data_frame_t frame = {
            .head = ROS_HEAD,
            .acce = *(vec3_t *)&g_ctx->mpu6050.curr_acce,
            .gyro = *(vec3_t *)&g_ctx->mpu6050.curr_acce,
            .velocity = {
                .x = g_ctx->wheel_perimeter * (r_rpm + l_rpm) / 2,
                .y = 0,
                .z = g_ctx->wheel_perimeter * (r_rpm - l_rpm) / g_ctx->wheel_space,
            },
            .power = 12 * 1000,
            .checksum = 0,
            .tail = ROS_TAIL,
        };
        uint8_t *sp = frame.buffer;
        uint8_t *ep = frame.buffer + offsetof(ros_send_data_frame_t, checksum);
        for (uint8_t *p = sp; p < ep; ++p) {
            frame.checksum ^= *p;
        }
        if (xSemaphoreTake(g_chassis_send_muetx, 0)) { 
            sendto(sock, frame.buffer, sizeof(frame), 0, (struct sockaddr *)&g_chassis_target_addr, sizeof(g_chassis_target_addr));
            xSemaphoreGive(g_chassis_send_muetx);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

static void ros_recv_task(void *pvParameters) {
    int sock = (int)pvParameters;
    uint8_t buffer[sizeof(ros_recv_data_frame_t) * 3];
    size_t len = 0;

    for (;;) {
        xSemaphoreTake(g_chassis_recv_muetx, portMAX_DELAY);
        ssize_t recvlen = recv(sock, buffer + len, sizeof(buffer) - len, 0);
        xSemaphoreGive(g_chassis_recv_muetx);
        if (recvlen <= 0) {
            g_ros_ctrl = false;
            continue;
        }
        len += recvlen;
        //ESP_LOGI(TAG, "Received %d bytes, total buffer len: %d", recvlen, len);
        //打印接收到的原始数据（十六进制）
        //ESP_LOGI(TAG, "Raw data: ");
        // for (int i = 0; i < recvlen && i < 32; i++) {
        //     printf("%02X ", buffer[len - recvlen + i]);
        //     if ((i + 1) % 8 == 0) printf(" ");
        //     if ((i + 1) % 16 == 0) printf("\n");
        // }
        // if (recvlen > 0) printf("\n");

        uint8_t *sp = buffer;
        uint8_t *ep = buffer + len;
        while (ep - sp >= sizeof(ros_recv_data_frame_t)) {
            if (!ros_recv_data_format_check(sp)) {
                ++sp;
            } else {
            ESP_LOGI(TAG, "接收一帧完整数据校验成功 ");
            vec3_t velocity;
            memcpy(&velocity, sp + offsetof(ros_recv_data_frame_t, velocity), sizeof(velocity));
            ESP_LOGI(TAG, "get velocity--x:%d, z:%d", velocity.x, velocity.z);
            float l_rpm = -(2 * velocity.x + g_ctx->wheel_space * velocity.z) / (2 * g_ctx->wheel_perimeter);
            float r_rpm = (2 * velocity.x - g_ctx->wheel_space * velocity.z) / (2 * g_ctx->wheel_perimeter);
            mc_set_expect_rpm(g_ctx->mc.left, l_rpm);
            mc_set_expect_rpm(g_ctx->mc.right, r_rpm);
            g_ros_ctrl = true;
            ESP_LOGI(TAG, "set left: %f, right: %f", l_rpm, r_rpm);
            sp += sizeof(ros_recv_data_frame_t);
            }
        }
        len = ep - sp;
        memmove(buffer, sp, len);
    }

    vTaskDelete(NULL);
}

static void lidar_send_task(void *pvParameters) {
    int sock = (int)pvParameters;
    uint8_t buffer[256];
    
    for (;;) {
        int len = uart_read_bytes(LIDAR_UART_NUM, buffer, sizeof(buffer), 0);
        if (len > 0 && g_has_lidar_target) { 
            if (xSemaphoreTake(g_lidar_send_muetx, 0)) {
                sendto(sock, buffer, len, 0, (struct sockaddr *)&g_lidar_target_addr, sizeof(g_lidar_target_addr));
                xSemaphoreGive(g_lidar_send_muetx);
                // ESP_LOGI(TAG, "Sent %d bytes of lidar data to %s:%d", 
                //     len, inet_ntoa(g_lidar_target_addr.sin_addr), ntohs(g_lidar_target_addr.sin_port));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static int json_parse(const char *raw, in_addr_t *ret_addr, in_port_t *ret_port) {
    cJSON *root = cJSON_Parse(raw);
    if (!root) return -1;

    int ret = -1;
    cJSON *item;

    if ((item = cJSON_GetObjectItem(root, "ip")) != NULL) {
        if (!cJSON_IsString(item)) goto cleanup;
        
        struct in_addr addr;
        if (inet_pton(AF_INET, item->valuestring, &addr) != 1) goto cleanup;//使用in将IP地址从字符串转换为网络字节顺序
        *ret_addr = addr.s_addr;
    }

    if (!(item = cJSON_GetObjectItem(root, "port")) || 
        !cJSON_IsNumber(item) || 
        (item->valuedouble < 0 || item->valuedouble > 65535)) {
        goto cleanup;
    }
    *ret_port = htons((in_port_t)item->valuedouble);

    if (!(item = cJSON_GetObjectItem(root, "type")) || 
        !cJSON_IsString(item) ||
        (strcmp(item->valuestring, "chassis") && strcmp(item->valuestring, "lidar") && strcmp(item->valuestring, "remote") )) {
        goto cleanup;
    }

    // 根据type返回对应的socket
    if (strcmp(item->valuestring, "chassis") == 0) {
        ret = g_chassis_sock;
    } else if (strcmp(item->valuestring, "lidar") == 0) {
        ret = g_lidar_sock;
    } else if (strcmp(item->valuestring, "remote") == 0) {
        ret = g_remote_sock;
    }

cleanup:
    cJSON_Delete(root);
    return ret;
}

static void conn_process_handler(void *pvParameters) {
    char buffer[256];
    int sock = (int)pvParameters;
    struct sockaddr_in addr;
    socklen_t socklen = sizeof(addr);
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
        (struct sockaddr *)&addr, &socklen);
    if (len > 0) {
        const char *type;
        buffer[len] = 0;
        ESP_LOGI(TAG, "Received config: %s", buffer); 
        int my_sock = json_parse(buffer, &addr.sin_addr.s_addr, &addr.sin_port);
        if (my_sock == -1) {
            ESP_LOGW(TAG, "parse json string failed");
            goto err;
        } else if (my_sock == g_chassis_sock) {
            xSemaphoreTake(g_chassis_send_muetx, portMAX_DELAY);
            xSemaphoreTake(g_chassis_recv_muetx, portMAX_DELAY);
            memcpy(&g_chassis_target_addr, &addr, sizeof(addr));
            xSemaphoreGive(g_chassis_recv_muetx);
            xSemaphoreGive(g_chassis_send_muetx);
            type = "chassis";
        } else if (my_sock == g_lidar_sock) {
            xSemaphoreTake(g_lidar_send_muetx, portMAX_DELAY);
            memcpy(&g_lidar_target_addr, &addr, sizeof(addr));
            g_has_lidar_target = true; 
            ESP_LOGI(TAG, "Lidar target address set to %s:%d", 
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
            xSemaphoreGive(g_lidar_send_muetx);
            type = "lidar";
        }
         else {
            xSemaphoreTake(g_remote_muetx, portMAX_DELAY);
            (void)connect(my_sock, (struct sockaddr *)&addr, sizeof(addr));
            xSemaphoreGive(g_remote_muetx);
            type = "remote";
        }
        char ip_str[IP4ADDR_STRLEN_MAX];
        inet_ntop(AF_INET, &addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "%s connect to %s:%"PRIu16, type, ip_str, ntohs(addr.sin_port));
        struct sockaddr_in addr;
        socklen_t socklen = sizeof(addr);
        getsockname(my_sock, (struct sockaddr *)&addr, &socklen);
        size_t len = snprintf(buffer, sizeof(buffer), "{\"port\": %"PRIu16"}", ntohs(addr.sin_port));
        printf("向PC发送端口%s\r\n",buffer);
        send(sock, buffer, len, 0);
    }
err:
    close(sock);
    vTaskDelete(NULL);
}

/**
 * @brief 用于设置udp套接字 g_chassis_sock 和 g_remote_sock 的连接对象，并向对方发送对应 udp 套接字绑定到的端口
 * 
 * @param pvParameters 为tcp server套接字，绑定到8080端口
 */
static void conn_task(void *pvParameters) {
    int sock = (int)pvParameters;
    listen(sock, 5);// 监听TCP连接
    const struct timeval timeout = {.tv_sec = 10, .tv_usec = 0};

    for (;;) {
        int c = accept(sock, NULL, NULL);// 接受客户端连接
        if (c != -1) {
            setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            xTaskCreate(conn_process_handler, "conn_process", 4096,
                (void *)c, 4, NULL);
        }
    }

    vTaskDelete(NULL);
}

static void sockfd_del(void) {
    const int *const end = g_socks + sizeof(g_socks) / sizeof(g_socks[0]);
    for (int *sock = g_socks; sock < end; ++sock) {
        if (*sock != -1) {
            close(*sock);
            *sock = -1;
        }
    }
}

static void mutex_del(void) {
    if (g_remote_muetx != NULL) {
        vSemaphoreDelete(g_remote_muetx);
        g_remote_muetx = NULL;
    }

    if (g_chassis_send_muetx != NULL) {
        vSemaphoreDelete(g_chassis_send_muetx);
        g_chassis_send_muetx = NULL;
    }

    if (g_lidar_send_muetx != NULL) {
        vSemaphoreDelete(g_lidar_send_muetx);
        g_lidar_send_muetx = NULL;
    }

}

static void task_del(void) {
    const TaskHandle_t *const end = g_task_handles + sizeof(g_task_handles) / sizeof(g_task_handles[0]);
    for (TaskHandle_t *handle = g_task_handles; handle < end; ++handle) {
        if (handle != NULL) {
            vTaskDelete(*handle);
            *handle = NULL;
        }
    }
}

esp_err_t start_wireless_conn(context_pack_t *ctx)
{
    esp_err_t ret;

    g_ctx = ctx;
    g_chassis_send_muetx = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(g_chassis_send_muetx != NULL, ESP_ERR_NO_MEM, err, TAG, "create ros mutex failed");
    g_chassis_recv_muetx = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(g_chassis_recv_muetx != NULL, ESP_ERR_NO_MEM, err, TAG, "create ros mutex failed");
    g_lidar_send_muetx = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(g_lidar_send_muetx != NULL, ESP_ERR_NO_MEM, err, TAG, "create lidar mutex failed");
  

    struct timeval timeout = { .tv_sec = 5, .tv_usec = 0 };

    struct sockaddr_in addr = {          // 定义esp32服务器端地址
        .sin_family = AF_INET,           // 使用IPv4地址族
        .sin_addr.s_addr = IPADDR_ANY,   // 监听所有可用网络接口（0.0.0.0）
        .sin_port = htons(8080),         // 绑定到端口8080（主机字节序小端转网络字节序大端）
    };
    g_conn_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//表示创建面向连接的TCP套接字
    ESP_GOTO_ON_FALSE(g_conn_sock != -1, ESP_FAIL, err, TAG, "create conn socket failed");
    ESP_GOTO_ON_FALSE(bind(g_conn_sock, (struct sockaddr *)&addr, sizeof(addr)) != -1,
        ESP_FAIL, err, TAG, "conn socket bind failed");//将套接字与指定的IP地址和端口（addr）关联，使套接字能够监听该端口的传入连接。

    addr.sin_port = htons(8082);
    g_chassis_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ESP_GOTO_ON_FALSE(g_chassis_sock != -1, ESP_FAIL, err, TAG, "create ros socket failed");
    ESP_GOTO_ON_FALSE(setsockopt(g_chassis_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != -1,
        ESP_FAIL, err, TAG, "set ros socket's timeout failed");
    ESP_GOTO_ON_FALSE(bind(g_chassis_sock, (struct sockaddr *)&addr, sizeof(addr)) != -1,
        ESP_FAIL, err, TAG, "ros socket bind failed");

    addr.sin_port = htons(8083);
    g_lidar_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ESP_GOTO_ON_FALSE(g_lidar_sock != -1, ESP_FAIL, err, TAG, "create lidar socket failed");
    ESP_GOTO_ON_FALSE(setsockopt(g_lidar_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != -1,
        ESP_FAIL, err, TAG, "set lidar socket's timeout failed");
    ESP_GOTO_ON_FALSE(bind(g_lidar_sock, (struct sockaddr *)&addr, sizeof(addr)) != -1,
        ESP_FAIL, err, TAG, "lidar socket bind failed");

    xTaskCreatePinnedToCore(conn_task, "conn",4096, (void *)g_conn_sock, 3, &g_task_handles[0],1);
    ESP_GOTO_ON_FALSE(g_task_handles[0] != NULL, ESP_FAIL, err, TAG, "create tasks failed");
    xTaskCreatePinnedToCore(ros_send_task, "ros_send",4096, (void *)g_chassis_sock, 2, &g_task_handles[1],1);
    ESP_GOTO_ON_FALSE(g_task_handles[1] != NULL, ESP_FAIL, err, TAG, "create tasks failed");
    xTaskCreatePinnedToCore(ros_recv_task, "ros_recv",4096, (void *)g_chassis_sock, 2, &g_task_handles[2],1);
    ESP_GOTO_ON_FALSE(g_task_handles[2] != NULL, ESP_FAIL, err, TAG, "create tasks failed");
    xTaskCreatePinnedToCore(lidar_send_task, "lidar_send",4096, (void *)g_lidar_sock, 2, &g_task_handles[3],1);
    ESP_GOTO_ON_FALSE(g_task_handles[3] != NULL, ESP_FAIL, err, TAG, "create tasks failed");

    
    return ESP_OK;

err:
    stop_wireless_conn();
    return ret;
}

esp_err_t stop_wireless_conn(void) {
    task_del();
    sockfd_del();
    mutex_del();
    return ESP_OK;
}
