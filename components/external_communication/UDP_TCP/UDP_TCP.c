#include "UDP_TCP.h"

#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include <sys/param.h>
#include "lidar.h"
#include "mdns.h"

#define TAG "UDP"
// #define ANOTC_DEBUG

#define MDNS_HOSTNAME "micu"//设备会去响应的主机名
#define MDNS_INSTANCE_NAME "ESP32"//设备名
#define HOST_IP_ADDR  "192.168.1.102 "//虚拟机IP地址
//#define HOST_IP_ADDR  "192.168.43.103 "//虚拟机IP地址

#define LIDAR_PORT 8888 //激光雷达端口
int sock_lidar;//lidar套接字
struct sockaddr_in dest_addr_lidar;//激光雷达套接字IP存储
bool lidar_state = false; //lidar连接状态


#define MOTION_PORT  8899 //motion端口
int sock_motion;//motion套接字
struct sockaddr_in dest_addr_motion;//motion套接字IP存储
bool motion_state = false; //motion连接状态


typedef enum
{
    UDP_CONNECT_STATUS_FALSE,
    UDP_CONNECT_STATUS_TRUE,
} udp_client_connect_status_t;

static int8_t is_udp_status_ok = UDP_CONNECT_STATUS_FALSE; // UDP连接化状态
static struct sockaddr_in dest_addr;                       // 目标地址
static struct sockaddr_storage source_addr;                // 当前地址
static socklen_t socklen = sizeof(source_addr);            // 地址长度
static int sock = -1;                                      // Server的SocketID
static proto_data_proto_mode_config_t* proto_config_t_ptr_;          // UDP通信模块配置


typedef struct {
    socklen_t len;// 地址长度
    struct sockaddr addr;// 网络地址（可以是IPv4或IPv6）
} sockaddr_pack_t;

static QueueHandle_t queue;// 队列句柄
static const UBaseType_t queue_len = 4; // 队列可以存放4个消息
static const UBaseType_t queue_size = sizeof(sockaddr_pack_t); // 每个消息的大小

static SemaphoreHandle_t mutex;					 // 用于保护共享资源(socket)的互斥锁，确保同一时间只有一个任务可以访问。

static EventGroupHandle_t events;				 // 事件组句柄
static const EventBits_t connect_event = 1 << 0; // 连接成功的事件位

//计算发送或接受的数据校验
//第一个值是校验前多少位，后面的模式为0是对接收的数据进行校验，为1是对发送的数据进行校验
uint8_t Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;

	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}

	//对接受到的数据进行校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

float XYZ_Target_Speed_transition(uint8_t High,uint8_t Low)
{
	//Data conversion intermediate variable
	short transition;

	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low);
	return
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s 
}

static void lidar_recv_task(void *pvParameters)   //数据接收任务
{

    char host_ip[] = HOST_IP_ADDR;  //ip地址
    int addr_family = 0;
    int ip_protocol = 0;


    while (1) 
    {
    	//定义IPV4  UDP通信参数
        dest_addr_lidar.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr_lidar.sin_family = AF_INET;
        dest_addr_lidar.sin_port = htons(LIDAR_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        //创建套接字
        sock_lidar =  socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock_lidar < 0) printf("lidar系统TCP套接字创建 失败\n");
        else printf("lidar系统TCP套接字创建 成功\n");

        printf("成功创建套接字 IP地址%s  端口号%d \n", host_ip, LIDAR_PORT);

        // 连接到ROS系统
        int err = connect(sock_lidar, (struct sockaddr *)&dest_addr_lidar, sizeof(struct sockaddr_in6));
        if (err != 0){
            	printf("lidar套接字连接 失败\n");
        }else {
            	printf("lidar套接字连接 成功\n");
        }
        while (1) 
        {

            int len = recv(sock_lidar, Receive_Data.buffer, sizeof(Receive_Data.buffer) - 1, pdMS_TO_TICKS(1));

            printf("%d  \n",len);
            // 接收时发生错误
            if (len < 0) {
            	break;
				printf("lidar接收失败\n");
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        if (sock_lidar != -1) {
        	printf("正在关闭lidar套接字并重新启动...\n");
            shutdown(sock_lidar, 0);
            close(sock_lidar);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

void motion_data_transition(void) //ROS数据发送前打包
{
		Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
		Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾

		//将编码器数值计算为xyz速度
		Send_Data.Sensor_Str.X_speed = ((uint16_t)(current_speeds[0]*1000+(uint16_t)current_speeds[1]*1000))/2;//单位mm/s
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((uint16_t)(current_speeds[1]*1000-(uint16_t)current_speeds[0]*1000))/MOTOR_WHEEL_SPACING;

		//printf(" x = %d z =  %d\n",Send_Data.Sensor_Str.X_speed,Send_Data.Sensor_Str.Z_speed);


		Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头

		//The three-axis speed of / / car is split into two eight digit Numbers
		//小车三轴速度,各轴都拆分为两个8位数据再发送
		Send_Data.buffer[1]=-Send_Data.Sensor_Str.X_speed >>8;
		Send_Data.buffer[2]=-Send_Data.Sensor_Str.X_speed ;
		Send_Data.buffer[3]=Send_Data.Sensor_Str.Y_speed>>8;
		Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed;
		Send_Data.buffer[5]=Send_Data.Sensor_Str.Z_speed >>8;
		Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed ;

		//修改
		//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
		//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送

		Send_Data.Sensor_Str.Accelerometer.Y_data = proto_imu_data.accel[1];
		Send_Data.buffer[7]=-Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
		Send_Data.buffer[8]=-Send_Data.Sensor_Str.Accelerometer.Y_data;

		Send_Data.Sensor_Str.Accelerometer.X_data = proto_imu_data.accel[0];
		Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data>>8;
		Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.X_data;

		Send_Data.Sensor_Str.Accelerometer.Z_data = proto_imu_data.accel[2];
		Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
		Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data;

		//The axis of the triaxial velocity of the / /imu is divided into two eight digits
		//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送

		Send_Data.Sensor_Str.Gyroscope.Y_data = proto_imu_data.gyro[1];
		Send_Data.buffer[13]=-Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
		Send_Data.buffer[14]=-Send_Data.Sensor_Str.Gyroscope.Y_data;

		Send_Data.Sensor_Str.Gyroscope.X_data = proto_imu_data.gyro[0];
		Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
		Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.X_data;

		Send_Data.Sensor_Str.Gyroscope.Z_data = proto_imu_data.gyro[2];
		Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
		Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data;

		//电池电压,拆分为两个8位数据发送
		Send_Data.Sensor_Str.Power_Voltage = 12*1000;
		Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8;
		Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage;


	  //Data check digit calculation, Pattern 1 is a data check
	  //数据校验位计算，模式1是发送数据校验
		Send_Data.buffer[22]=Check_Sum(19,1);

		Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾

		TCP_write_motion(Send_Data.buffer,24);//ros系统TCP发送
}

static void motion_send_task(void *pvParameters)   //数据发送任务
{

    while (1) 
    {
    	motion_data_transition(); //ROS数据发送前打包
    	vTaskDelay(pdMS_TO_TICKS(50));//以20hz的速度发送

    }
}

static void motion_rec_task(void *pvParameters)   //数据接收任务
{
    char rx_buffer[128]; //接受缓冲区
    char host_ip[] = HOST_IP_ADDR;  //ip地址
    int addr_family = 0;
    int ip_protocol = 0;


    while (1) {

    	//定义IPV4  TCP通信参数
        struct sockaddr_in dest_addr_motion;
        dest_addr_motion.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr_motion.sin_family = AF_INET;
        dest_addr_motion.sin_port = htons(MOTION_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        //创建套接字
        sock_motion =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock_motion < 0) printf("motion系统TCP套接字创建 失败\n");
        else printf("motion系统TCP套接字创建 成功\n");

        printf("成功创建motion套接字 IP地址%s  端口号%d \n", host_ip, MOTION_PORT);

         int err = connect(sock_motion, (struct sockaddr *)&dest_addr_motion, sizeof(struct sockaddr_in6));
           if (err != 0){
            	printf("motion套接字连接 失败\n");
            }else {
            	printf("motion套接字连接 成功\n");
            }

        while (1) 
		{

            int len = recv(sock_motion, Receive_Data.buffer,11,NULL);

            printf("%d    ",len);
            // 接收时发生错误
            if (len < 0) {
            	break;
				printf("motion接收失败\n");
            }
			else
			{
            		if(Receive_Data.buffer[9] ==Check_Sum(9,0))//验证前9位数据
               	       {
               	           //计算三轴对应的数据
               	           Receive_Data.Control_Str.X_speed=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
               	           Receive_Data.Control_Str.Y_speed=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
               	           Receive_Data.Control_Str.Z_speed=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
               	           printf("成功接收ROS数据 ：x %0.2f   y %0.2f   z   %0.2f   \n",Receive_Data.Control_Str.X_speed,Receive_Data.Control_Str.Y_speed,Receive_Data.Control_Str.Z_speed);
               	       }
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        if (sock_motion != -1) {
        	printf("正在关闭motion套接字并重新启动...\n");
            shutdown(sock_motion, 0);
            close(sock_motion);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

void TCP_write_motion(uint8_t *data,uint8_t len)//motion系统TCP发送
{

		//发送数据
		int err = send(sock_motion, data, len, 0);
		if (err < 0) motion_state = false;//未连接motion
		else motion_state = true;//已连接motion
}

void udp_write_lidar(uint8_t *data,uint8_t len)//lidar激光雷达UDP发送
{
	//发送数据
	int err = send(sock_lidar, data, len, 0);
	if (err < 0) 
	{
		printf("lidar发送失败\n");
		lidar_state = false;//lidar未连接
	}
	else 
	{
		printf("lidar发送成功\n");
		lidar_state = true;//已连接lidar
	}

}


esp_err_t start_mdns_service(void) {
	// 初始化 mDNS 服务

	esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(mdns_hostname_set(MDNS_HOSTNAME), TAG, "");// 设置 hostname
    ESP_RETURN_ON_ERROR(mdns_instance_name_set(MDNS_INSTANCE_NAME), TAG, "");// 设置实例名

    return ESP_OK;
}

int udp_socket_create(void) {
    const struct sockaddr_in local_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),//指定IP 地址，INADDR_ANY 表示监听所有可用的网络接口
        .sin_family = AF_INET,//指定地址族，AF_INET 表示使用 IPv4。
        .sin_port = htons(LIDAR_PORT),//指定端口号
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);//SOCK_DGRAM 表示创建一个UDP套接字
    ESP_RETURN_ON_FALSE(sock >= 0, sock, TAG, "套接字创建失败: errno %d", errno);
    //sock >= 0是要检查的条件。如果 sock 小于 0，说明套接字创建失败。
    const struct timeval timeout = {.tv_sec = 10, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));//设置套接字选项，SOL_SOCKET 表示设置套接字级别的选项，SO_RCVTIMEO 表示设置接收超时时间。
    
    int err = bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));//将套接字绑定到指定的本地地址和端口
    ESP_RETURN_ON_FALSE(err == ERR_OK, sock, TAG, "Socket unable to bind: errno %d", errno);
    ESP_LOGI(TAG, "成功创建套接字，端口号 %d", LIDAR_PORT);

    return sock;
}



static void conn_task(void *pvParameters) {
    int sock = (intptr_t)pvParameters;// 从参数获取socket描述符
    sockaddr_pack_t pack;
    char ip_str[INET6_ADDRSTRLEN];

    if (sock < 0) {
        ESP_LOGE(TAG, "套接字创建失败");
        vTaskDelete(NULL);
    }

    while (true) {
        while (xQueueReceive(queue, &pack, portMAX_DELAY) != pdPASS);// 1. 等待接收新的连接地址

        while (xSemaphoreTake(mutex, portMAX_DELAY) != pdPASS);// 2. 获取互斥锁，保护连接操作
        ESP_LOGI(TAG, "开始连接...");
        while (true) {
            if (connect(sock, &pack.addr, pack.len) == ERR_OK) { // 3. 尝试建立连接
                const void *numeric_addr;
                in_port_t port;
				// 4. 连接成功，解析并打印地址信息
                if (pack.addr.sa_family == AF_INET) {// IPv4
                    numeric_addr = &((struct sockaddr_in *)&pack.addr)->sin_addr;
                    port = ((struct sockaddr_in *)&pack.addr)->sin_port;
                } else {// IPv6
                    numeric_addr = &((struct sockaddr_in6 *)&pack.addr)->sin6_addr;
                    port = ((struct sockaddr_in6 *)&pack.addr)->sin6_port;
                }
                inet_ntop(pack.addr.sa_family, numeric_addr, ip_str, pack.len);
                ESP_LOGI(TAG, "获取到主机地址 %s:%d", ip_str, ntohs(port));
                xEventGroupSetBits(events, connect_event);// 5. 设置连接事件，通知其他任务连接已建立
                break;
            }
            ESP_LOGE(TAG, "套接字连接失败: errno %d", errno);
            ESP_LOGI(TAG, "尝试在5秒后重新连接...");
            usleep(1000 * 5000);
        }
        xSemaphoreGive(mutex);
    }

    vTaskDelete(NULL);
}

static void send_task(void *pvParameters) {
    static uint8_t buffer[128];
    int sock = (intptr_t)pvParameters;

    if (sock < 0) {
        ESP_LOGE(TAG, "socket套接字创建 失败\n");
        vTaskDelete(NULL);
    }

    while (xEventGroupWaitBits(events, connect_event, pdTRUE, pdTRUE, portMAX_DELAY) != pdPASS);// 等待连接事件，直到事件发生

    ESP_LOGI(TAG, "发送任务开始");
    while (true) {
        int len = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer), 0);
        if (len > 0 && xSemaphoreTake(mutex, 0) == pdPASS) {// 如果读取到数据且成功获取互斥量
            int err = send(sock, buffer, len, 0);
            if (err < 0) ESP_LOGE(TAG, "发送失败: errno %d", errno);
            else ESP_LOGI(TAG, "发送成功");
            xSemaphoreGive(mutex);// 释放互斥量
        }

        usleep(1000);
    }

    vTaskDelete(NULL);
}

static void recv_task(void *pvParameters) {
    int sock = (intptr_t)pvParameters;
    sockaddr_pack_t pack;

    if (sock < 0) {
        ESP_LOGE(TAG, "socket套接字创建 失败\n");
        vTaskDelete(NULL);
    }

    while (true) {
        in_port_t port;
        pack.len = sizeof(pack.addr); // 设置地址结构体长度
		// 接收数据（这里主要接收端口信息）
        // sock: socket句柄
        // &port: 接收缓冲区
        // sizeof(port): 接收长度
        // 0: 标志位
        // &pack.addr: 发送方地址结构
        // &pack.len: 地址结构长度
        int len = recvfrom(sock, &port, sizeof(port), 0, &pack.addr, &pack.len);
		printf("接收数据长度：%d \n",len);
        if (len != sizeof(port)) { // 如果接收的数据长度不正确，继续循环
            continue;
        }
         // 根据地址类型设置端口号
        if (pack.addr.sa_family == AF_INET) {// IPv4
            ((struct sockaddr_in *)&pack.addr)->sin_port = port;
        } else {// IPv6
            ((struct sockaddr_in6 *)&pack.addr)->sin6_port = port;
        }

        xQueueSend(queue, &pack, 0);// 将接收到的数据包添加到队列中
    }

    vTaskDelete(NULL);
}


bool UDP_init(int *sock)
{
    // 初始化MDNS
    ESP_RETURN_ON_ERROR(start_mdns_service(), TAG, "mdns start failed");

    // 创建socket
    while (true) {
        *sock = udp_socket_create();
        if (*sock >= 0) {
            break;
        }
        ESP_LOGE(TAG, "5秒后重新创建套接字...");
        usleep(1000 * 5000);
    }
  	return true;
}

esp_err_t UDP_task(int sock)
{
	mutex = xSemaphoreCreateMutex();
    queue = xQueueCreate(queue_len, queue_size);
    events = xEventGroupCreate();
    
    if (mutex == NULL || queue == NULL || events == NULL) {
        ESP_LOGE(TAG, "创建RTOS资源失败\n");
        return ESP_ERR_NO_MEM;
    }

    // 创建通信任务
    void *pvParameters = (void *)(uintptr_t)sock;

	#ifdef ANOTC_DEBUG
		socket_anotc_init();//创建与地面站通信的套接字
		xTaskCreate(udp_anotc_read_task, "udp_anotc_read_task", 1024*8, (void*)AF_INET, 2, NULL);//创建接收地面站数据的任务
	#else
		xTaskCreate(send_task, "send", 4096, pvParameters, 6, NULL);
		xTaskCreate(recv_task, "receive", 4096, pvParameters, 7, NULL);
		xTaskCreate(conn_task, "connect", 4096, pvParameters, 5, NULL);
		//xTaskCreatePinnedToCore(lidar_recv_task, "lidar_recv_task", 1024*8, (void*)AF_INET, 15, NULL,0);
		// xTaskCreatePinnedToCore(motion_rec_task, "motion_rec_task", 1024*8,(void*)AF_INET,7, NULL,1);  //创建接收ros系统数据的任务
		// xTaskCreatePinnedToCore(motion_send_task, "motion_send_task", 1024*8, (void*)AF_INET, 14, NULL,1);
	#endif
    
    return ESP_OK;  // 成功时返回ESP_OK
}

