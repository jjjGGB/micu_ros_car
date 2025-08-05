# ESP32-ROS机器人通信协议详细文档

## 目录

1. [系统概述](#1-系统概述)
2. [网络通信基础知识](#2-网络通信基础知识)
3. [系统架构设计](#3-系统架构设计)
4. [通信协议详解](#4-通信协议详解)
5. [连接建立流程](#5-连接建立流程)
6. [数据传输机制](#6-数据传输机制)
7. [虚拟串口技术](#7-虚拟串口技术)
8. [错误处理与可靠性](#8-错误处理与可靠性)
9. [性能优化](#9-性能优化)
10. [实际应用示例](#10-实际应用示例)

---

## 1. 系统概述

### 1.1 整体架构

本系统是一个基于ESP32的机器人与虚拟机ROS系统之间的完整通信解决方案。采用**混合TCP/UDP通信架构**：

```
┌─────────────────┐    WiFi网络     ┌─────────────────┐    虚拟串口    ┌─────────────────┐
│   ESP32端       │ ←──────────→   │   虚拟机端       │ ←──────────→  │   ROS系统       │
│ (wireless_conn.c)│               │ (start_comm.py) │               │ (launch文件)    │
├─────────────────┤               ├─────────────────┤               ├─────────────────┤
│ • TCP服务器     │               │ • TCP客户端     │               │ • chassis_drive │
│ • UDP服务器     │               │ • UDP客户端     │               │ • ydlidar_driver│
│ • 多任务管理    │               │ • 虚拟串口      │               │ • 导航算法      │
│ • 数据处理      │               │ • 数据转发      │               │ • 路径规划      │
└─────────────────┘               └─────────────────┘               └─────────────────┘
```

### 1.2 通信特点

- **配置管理**：TCP确保连接参数可靠传输
- **数据传输**：UDP保证实时性和高吞吐量
- **透明桥接**：虚拟串口实现网络到串口的无缝转换
- **多设备支持**：同时支持chassis和lidar等多种设备类型

---

## 2. 网络通信基础知识

### 2.1 TCP协议详解

#### 2.1.1 TCP基本特性

**传输控制协议(TCP)**是一种面向连接的、可靠的传输层协议：

**核心特性：**

- **面向连接**：在传输数据之前，TCP需要在通信的双方之间建立一个连接，这个
  过程通常通过“三次握手”完成。
- **可靠传输**：TCP通过确认机制、序列号、超时重传、校验和、流量控制、拥塞
  控制等技术手段来保证数据传输的可靠性。
- 字节流服务：TCP将应用程序传输的数据看作一个无结构的字节流，而不考虑
  这些字节属于什么样的结构（如消息、记录等）。
- **全双工通信**：TCP允许通信双方同时发送和接收数据，这意味着数据在两个方向上可以同时传输。

**技术机制：**

```
TCP连接建立（三次握手）：
客户端 → 服务器: SYN (seq=x)
服务器 → 客户端: SYN+ACK (seq=y, ack=x+1)
客户端 → 服务器: ACK (ack=y+1)
连接建立完成
```

#### 2.1.2 在本系统中的应用

```c
// ESP32端TCP服务器创建
g_conn_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
struct sockaddr_in addr = {         //声明用于存储地址信息的结构体
    .sin_family = AF_INET,
    .sin_addr.s_addr = IPADDR_ANY,  // 监听所有网络接口
    .sin_port = htons(8080),        // 配置端口
};
bind(g_conn_sock, (struct sockaddr *)&addr, sizeof(addr));
listen(sock, 5);  // 监听TCP连接，最多5个排队连接
```

```python
# 虚拟机端TCP客户端连接
def connect_smart_car(local_port, toaddr, device_type):
    message = json.dumps({
        "port": local_port,
        "type": device_type
    })
    with socket.socket() as s:  # 创建TCP连接
        s.connect(toaddr)       # TCP连接到ESP32的8080端口
        s.send(message.encode("ascii"))  # 发送JSON消息
        response = s.recv(1024)          # 接收响应
```

### 2.2 UDP协议详解

#### 2.2.1 UDP基本特性

**用户数据报协议(UDP)**是一种无连接的、不可靠的传输层协议：

**核心特性：**

- **无连接**：无需建立连接即可发送数据
- **不可靠传输**：不保证数据到达和顺序
- **无流量控制**：发送方可任意速率发送
- **轻量级**：协议开销小，头部仅8字节
- **支持广播/组播**：一对多通信

**性能对比：**

| 特性     | TCP         | UDP       | 本系统应用                 |
| -------- | ----------- | --------- | -------------------------- |
| 连接建立 | 需要        | 不需要    | 配置用TCP，数据用UDP       |
| 传输延迟 | 10-50ms     | 1-5ms     | 实时数据需要低延迟         |
| 带宽开销 | 高(20+字节) | 低(8字节) | 传感器数据量大             |
| 可靠性   | 高          | 低        | 配置需可靠，数据可容忍丢失 |

#### 2.2.2 在本系统中的应用

```c
// ESP32端UDP服务器创建
g_chassis_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
addr.sin_port = htons(8082);  // chassis数据端口
bind(g_chassis_sock, (struct sockaddr *)&addr, sizeof(addr));

g_lidar_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
addr.sin_port = htons(8083);  // lidar数据端口
bind(g_lidar_sock, (struct sockaddr *)&addr, sizeof(addr));
```

```python
# 虚拟机端UDP客户端
def socket_to_serial(self):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('0.0.0.0', 0))  # 绑定到随机UDP端口
        port = sock.getsockname()[1]

        while self.running:
            # 接收ESP32发来的UDP数据
            message, self.last_addr = sock.recvfrom(1024)
            # 写入虚拟串口，传递给ROS
            self.seri.write(message)
```

### 2.3 Socket编程

    如果你想和远方的朋友通电话，但是，没有办法直接把自己的声音放在电线上变成电流信号，你需要使用电话机拿起听筒拨号，而这个电话就是Socket，它让你简单方便地完了电流通话。从我们编程的角度来看，我们直接使用TCP传输信息，需要考虑的东西太多了，而Scoket替我们封装实现了TCP，我们只需使用Socket的APl，即可间接完成TCP通信。

    Socket是网络通信的“接口”，也就是程序和网络之间的中介。它为开发者提供了一套通用的函数或API，允许程序通过网络发送和接收数据。具体地说，Socket抽象了网络底层的细节，比如数据包的传输、协议的选择、错误处理等，开发者只需要使用Socket提供的函数来建立连接和传输数据，而不需要关心网络底层的实现。

#### 2.3.1 Socket生命周期

![1750336484385](image/ros_protocol/1750336484385.png)

```
Socket编程流程：
1. socket()    - 创建套接字
2. bind()      - 绑定地址和端口
3. listen()    - 监听连接（仅TCP）
4. accept()    - 接受连接（仅TCP）
5. send/recv() - 数据传输
6. close()     - 关闭连接
```

#### 2.3.2 地址结构详解

```c
struct sockaddr_in {
    sa_family_t    sin_family;  // 地址族，AF_INET表示IPv4
    in_port_t      sin_port;    // 端口号，网络字节序
    struct in_addr sin_addr;    // IP地址
    char           sin_zero[8]; // 填充字节，保持与sockaddr相同大小
};
```

**字节序转换：**

```c
// 主机字节序转网络字节序
uint16_t port_network = htons(8080);    // host to network short
uint32_t addr_network = htonl(INADDR_ANY); // host to network long

// 网络字节序转主机字节序
uint16_t port_host = ntohs(addr.sin_port);  // network to host short
```

---

## 3. 系统架构设计

### 3.1 ESP32端架构 (wireless_conn.c)

#### 3.1.1 核心组件

```c
// 全局socket数组
static int g_socks[] = { -1, -1, -1, -1 };
#define g_conn_sock (g_socks[0])      // TCP配置socket (端口8080)
#define g_chassis_sock (g_socks[1])   // UDP chassis socket (端口8082)
#define g_remote_sock (g_socks[2])    // UDP remote socket (动态分配)
#define g_lidar_sock (g_socks[3])     // UDP lidar socket (端口8083)

// 任务句柄数组
static TaskHandle_t g_task_handles[4];

// 互斥锁保护
static SemaphoreHandle_t g_chassis_send_muetx;
static SemaphoreHandle_t g_chassis_recv_muetx;
static SemaphoreHandle_t g_lidar_send_muetx;
```

#### 3.1.2 任务架构

```
ESP32任务架构：
├── conn_task (优先级3)
│   ├── 监听TCP连接 (端口8080)
│   ├── 接受客户端连接
│   └── 创建conn_process_handler处理配置
├── ros_send_task (优先级2)
│   ├── 读取传感器数据
│   ├── 构造数据帧
│   └── UDP发送到虚拟机 (50ms周期)
├── ros_recv_task (优先级2) [可选]
│   ├── 接收控制命令
│   └── 解析并执行
└── lidar_send_task (优先级2)
    ├── 从UART读取激光雷达数据
    └── UDP转发到虚拟机 (10ms周期)
```

#### 3.1.3 数据流向

```
ESP32数据流：
传感器数据 → UART → ESP32 → UDP → 虚拟机 → 虚拟串口 → ROS
控制命令 ← UART ← ESP32 ← UDP ← 虚拟机 ← 虚拟串口 ← ROS
```

### 3.2 虚拟机端架构 (start_comm.py)

#### 3.2.1 核心组件

```python
class Communication:
    def __init__(self, serial_port: str, tcpaddr, device_type: str):
        self.seri = serial.Serial(serial_port, timeout=0.01)  # 虚拟串口
        self.tcpaddr = tcpaddr          # ESP32的TCP地址
        self.device_type = device_type  # "chassis" 或 "lidar"
        self.last_addr = None          # UDP通信的目标地址
        self.running = True            # 运行状态标志
```

#### 3.2.2 多线程架构

```
虚拟机线程架构：
├── socket_to_serial线程
│   ├── 创建UDP socket并绑定随机端口
│   ├── 通过TCP发送配置信息给ESP32
│   ├── 接收ESP32的UDP数据
│   └── 写入虚拟串口传递给ROS
└── serial_to_socket线程
    ├── 从虚拟串口读取ROS数据
    ├── 累积数据包
    └── 通过UDP发送到ESP32
```

#### 3.2.3 设备管理

```python
# 多设备通信管理
comms = {
    "lidar": Communication(
        serial_ports["lidar"][0],      # 虚拟串口设备端
        ("micu-ros-car.local", 8080),  # ESP32 TCP地址
        "lidar",                       # 设备类型
    ),
    "chassis": Communication(
        serial_ports["chassis"][0],    # 虚拟串口设备端
        ("micu-ros-car.local", 8080),  # ESP32 TCP地址
        "chassis"                      # 设备类型
    )
}
```

---

## 4. 通信协议详解

### 4.1 TCP配置协议

#### 4.1.1 JSON配置消息格式

虚拟机端发送给ESP32的配置信息：

```json
{
    "ip": "192.168.1.100",    // 可选，客户端IP地址
    "port": 8082,             // 必需，UDP端口号
    "type": "chassis"         // 必需，设备类型："chassis"/"lidar"/"remote"
}
```

#### 4.1.2 设备类型映射

| 设备类型    | ESP32端口 | 功能描述         | 数据特征       |
| ----------- | --------- | ---------------- | -------------- |
| `chassis` | 8082      | 底盘控制数据通信 | 双向，50ms周期 |
| `lidar`   | 8083      | 激光雷达数据传输 | 单向，10ms周期 |
|             |           |                  |                |

#### 4.1.3 ESP32端配置解析

```c
static int json_parse(const char *raw, in_addr_t *ret_addr, in_port_t *ret_port) {
    cJSON *root = cJSON_Parse(raw);
    if (!root) return -1;

    // 解析端口号
    cJSON *item = cJSON_GetObjectItem(root, "port");
    if (!cJSON_IsNumber(item) ||
        (item->valuedouble < 0 || item->valuedouble > 65535)) {
        goto cleanup;
    }
    *ret_port = htons((in_port_t)item->valuedouble);

    // 解析设备类型并返回对应socket
    item = cJSON_GetObjectItem(root, "type");
    if (strcmp(item->valuestring, "chassis") == 0) {
        return g_chassis_sock;
    } else if (strcmp(item->valuestring, "lidar") == 0) {
        return g_lidar_sock;
    } else if (strcmp(item->valuestring, "remote") == 0) {
        return g_remote_sock;
    }

cleanup:
    cJSON_Delete(root);
    return -1;
}
```

### 4.2 UDP数据传输协议

#### 4.2.1 数据包格式

- **无特定格式**：直接透传原始数据
- **最大包长**：1024字节
- **传输频率**：
  - 激光雷达：10ms周期（100Hz）
  - 底盘数据：50ms周期（20Hz）

#### 4.2.2 数据流向详解

```
上行数据流（ESP32 → 虚拟机 → ROS）：
┌─────────┐    UART     ┌─────────┐    UDP      ┌─────────┐    虚拟串口   ┌─────────┐
│ 传感器  │ ──────────→ │  ESP32  │ ──────────→ │ 虚拟机  │ ──────────→ │   ROS   │
│ (Lidar) │             │         │             │         │             │ (ydlidar)│
└─────────┘             └─────────┘             └─────────┘             └─────────┘

下行数据流（ROS → 虚拟机 → ESP32）：
┌─────────┐   虚拟串口   ┌─────────┐    UDP      ┌─────────┐    控制     ┌─────────┐
│   ROS   │ ──────────→ │ 虚拟机  │ ──────────→ │  ESP32  │ ──────────→ │ 电机控制│
│(chassis)│             │         │             │         │             │         │
└─────────┘             └─────────┘             └─────────┘             └─────────┘
```

---

## 5. 连接建立流程

### 5.1 完整连接时序图

```
虚拟机端                ESP32端
    |                      |
    |  1. mDNS解析         |
    |  micu-ros-car.local  |
    |                      |
    |  2. TCP连接建立      |
    |====8080=============>|
    |                      |
    |  3. JSON配置发送     |
    |  {"port":12345,      |
    |   "type":"chassis"}  |
    |--------------------->|
    |                      | 4. 解析配置
    |                      | 5. 保存目标地址
    |                      | 6. 获取本地端口
    |  7. 端口响应         |
    |  {"port":8082}       |
    |<---------------------|
    |                      |
    |  8. UDP数据传输开始  |
    |<====8082/8083=======>|
    |                      |
```

### 5.2 详细连接步骤

#### 步骤1：mDNS设备发现

```python
# 虚拟机端通过mDNS发现ESP32设备
tcpaddr = ("micu-ros-car.local", 8080)
```

**mDNS工作原理：**

- **多播DNS**：在局域网内通过多播方式解析域名
- **零配置**：无需手动配置DNS服务器
- **自动发现**：设备可以自动发现网络中的服务

#### 步骤2-3：TCP连接和配置发送

```python
def connect_smart_car(local_port, toaddr, device_type):
    message = json.dumps({
        "port": local_port,
        "type": device_type
    })
    with socket.socket() as s:
        try:
            s.connect(toaddr)           # TCP连接到ESP32:8080
            s.send(message.encode("ascii"))  # 发送JSON配置
            response = s.recv(1024)     # 接收响应
            print(f"Received response: {response.decode()}")
        except socket.error as e:
            raise RuntimeError(f"Connection failed: {toaddr[0]}") from e
```

---

## 6. 数据传输机制

### 6.1 激光雷达数据传输

#### 6.1.1 ESP32端发送逻辑

```c
static void lidar_send_task(void *pvParameters) {
    int sock = (int)pvParameters;
    uint8_t buffer[256];

    for (;;) {
        // 从UART读取激光雷达数据
        int len = uart_read_bytes(LIDAR_UART_NUM, buffer, sizeof(buffer), 0);

        if (len > 0 && g_has_lidar_target) {
            if (xSemaphoreTake(g_lidar_send_muetx, 0)) {
                // 通过UDP发送到虚拟机
                sendto(sock, buffer, len, 0,
                    (struct sockaddr *)&g_lidar_target_addr,
                    sizeof(g_lidar_target_addr));
                xSemaphoreGive(g_lidar_send_muetx);
                ESP_LOGI(TAG, "Sent %d bytes of lidar data", len);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms高频传输
    }
}
```

#### 6.1.2 虚拟机端接收逻辑

```python
def socket_to_serial(self):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('0.0.0.0', 0))  # 绑定到随机UDP端口
        port = sock.getsockname()[1]
        sock.settimeout(5)

        # 通过TCP告诉ESP32这个UDP端口
        connect_smart_car(port, self.tcpaddr, self.device_type)

        while self.running:
            try:
                # 接收ESP32的UDP数据
                message, self.last_addr = sock.recvfrom(1024)
                # 写入虚拟串口，传递给ROS
                self.seri.write(message)
                print(f"Received {len(message)} bytes from ESP32")
            except socket.timeout:
                # 超时处理和重连
                connect_smart_car(port, self.tcpaddr, self.device_type)
                print(f"Reconnecting to {self.tcpaddr}")
```

---

## 7. 虚拟串口技术

### 7.1 socat工具详解

#### 7.1.1 socat基本概念

**socat (Socket CAT)** 是一个强大的网络工具，可以在两个独立的数据通道之间建立双向数据传输。

**核心功能：**

- **数据中继**：在不同类型的数据流之间转发数据
- **协议转换**：支持多种协议和数据格式
- **虚拟设备**：创建虚拟的网络设备和串口设备
- **双向通信**：支持全双工数据传输

#### 7.1.2 虚拟串口创建

```python
# 创建虚拟串口对
serial_ports = {
    "lidar": (f"/{pts_dir}/pts0", f"/{pts_dir}/pts1"),
    "chassis": (f"/{pts_dir}/pts2", f"/{pts_dir}/pts3"),
}

# 使用socat创建虚拟串口
args = lambda item: ["socat",
    f"pty,raw,echo=0,link={item[0]}",
    f"pty,raw,echo=0,link={item[1]}"]
vserial_popens = [sp.Popen(args(item), stdout=sp.DEVNULL, stderr=sp.DEVNULL)
                  for item in serial_ports.values()]
```

**socat参数详解：**

- `pty`：创建伪终端(pseudo-terminal)
- `raw`：原始模式，不进行任何数据处理
- `echo=0`：禁用回显
- `link=path`：创建符号链接到指定路径

#### 7.1.3 虚拟串口工作原理

```
虚拟串口对工作原理：
┌─────────────┐    socat进程    ┌─────────────┐
│   pts0      │ ←──────────→   │    pts1     │
│ (设备端)    │                │  (ROS端)    │
└─────────────┘                └─────────────┘
      ↑                              ↑
      │                              │
┌─────────────┐                ┌─────────────┐
│ Python脚本  │                │ ROS驱动程序 │
│ 网络数据    │                │ 串口通信    │
└─────────────┘                └─────────────┘
```

### 7.2 ROS配置文件动态修改

#### 7.2.1 launch文件参数替换

```python
# 替换chassis配置文件中的串口参数
pattern = r'(<\s*param\s+name="usart_port_name"\s+type="string"\s+value=")([^"]*)("\s*/>)'
repl = rf'\g<1>{serial_ports["chassis"][1]}\g<3>'
with open(chassis_config_path) as f:
    s = f.read()
with open(chassis_config_path, "w") as f:
    f.write(re.sub(pattern, repl, s))

# 替换lidar配置文件中的串口参数
pattern = r'(<\s*param\s+name="port"\s+type="string"\s+value=")([^"]*)("\s*/>)'
repl = rf'\g<1>{serial_ports["lidar"][1]}\g<3>'
with open(lidar_config_path) as f:
    s = f.read()
with open(lidar_config_path, "w") as f:
    f.write(re.sub(pattern, repl, s))
```

**正则表达式解析：**

- `<\s*param\s+name="usart_port_name"`：匹配param标签开始
- `type="string"\s+value="`：匹配类型和value属性
- `([^"]*)`：捕获当前的串口路径值
- `"\s*/>`：匹配标签结束

#### 7.2.2 配置文件示例

**修改前的chassis_start.launch：**

```xml
<param name="usart_port_name" type="string" value="/dev/ttyUSB0" />
```

**修改后的chassis_start.launch：**

```xml
<param name="usart_port_name" type="string" value="/tmp/pts/pts3" />
```

---

## 2. 数据帧格式

### 2.1 小车发送数据帧 (ros_send_data_frame_t)

小车每50ms向ROS系统发送一次状态数据，包含IMU数据和运动学信息。

#### 2.1.1 数据结构

| 字段名   | 类型    | 大小(字节) | 描述                                     |
| -------- | ------- | ---------- | ---------------------------------------- |
| head     | uint8_t | 1          | 帧头标识，固定值为0x7B                   |
| reserve  | uint8_t | 1          | 保留字段                                 |
| velocity | vec3_t  | 6          | 三轴速度，x=线速度，y=0，z=角速度        |
| acce     | vec3_t  | 6          | 三轴加速度，来自MPU6050                  |
| gyro     | vec3_t  | 6          | 三轴角速度，来自MPU6050                  |
| power    | int16_t | 2          | 电源电压(mV)，固定值12000                |
| checksum | uint8_t | 1          | 校验和，从帧头到校验和前所有字节的异或值 |
| tail     | uint8_t | 1          | 帧尾标识，固定值为0x7D                   |

总大小：24字节

#### 2.1.2 vec3_t结构

| 字段名 | 类型    | 大小(字节) | 描述    |
| ------ | ------- | ---------- | ------- |
| x      | int16_t | 2          | X轴数据 |
| y      | int16_t | 2          | Y轴数据 |
| z      | int16_t | 2          | Z轴数据 |

#### 2.1.3 速度计算方法

```c
velocity.x = wheel_perimeter * (right_rpm + left_rpm) / 2;  // 线速度
velocity.y = 0;  // 横向速度恒为0（差速驱动无法横向移动）
velocity.z = wheel_perimeter * (right_rpm - left_rpm) / wheel_space;  // 角速度
```

其中：

- `wheel_perimeter`：车轮周长(m)
- `right_rpm`：右轮实际转速(RPM)
- `left_rpm`：左轮实际转速(RPM)
- `wheel_space`：两轮间距(m)

#### 2.1.4 校验和计算

校验和使用异或(XOR)运算，计算从帧头到校验和字段前的所有字节：

```c
uint8_t checksum = 0;
uint8_t *sp = frame.buffer;
uint8_t *ep = frame.buffer + offsetof(ros_send_data_frame_t, checksum);
for (uint8_t *p = sp; p < ep; ++p) {
    checksum ^= *p;
}
```

### 2.2 ROS系统发送数据帧 (ros_recv_data_frame_t)

ROS系统向小车发送控制命令，设置期望速度。

#### 2.2.1 数据结构

| 字段名   | 类型     | 大小(字节) | 描述                                     |
| -------- | -------- | ---------- | ---------------------------------------- |
| head     | uint8_t  | 1          | 帧头标识，固定值为0x7B                   |
| reserve  | uint16_t | 2          | 保留字段                                 |
| velocity | vec3_t   | 6          | 期望三轴速度，单位为m/s的1000倍          |
| checksum | uint8_t  | 1          | 校验和，从帧头到校验和前所有字节的异或值 |
| tail     | uint8_t  | 1          | 帧尾标识，固定值为0x7D                   |

总大小：11字节

## 3. 通信流程

### 3.1 连接建立

1. ROS系统通过TCP连接到小车的8080端口
2. ROS系统发送JSON格式的配置信息，包含：
   ```json
   {
     "port": 12345,  // ROS系统用于接收数据的UDP端口
     "type": "chassis"  // 连接类型，可以是"chassis"或"lidar"
   }
   ```
3. 小车解析配置信息，记录ROS系统的IP地址和端口
4. 小车开始向指定地址发送数据

### 3.2 数据传输

1. 小车每50ms发送一次状态数据帧
2. ROS系统可随时发送控制命令
3. 通信采用UDP协议，无需保持连接状态

### 3.3 错误处理

1. 如果接收到的数据帧校验和错误，则丢弃该帧
2. 如果连续多次未收到对方数据，可以重新发起连接请求

## 4. 示例代码

### 4.1 发送数据帧

```c
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

// 计算校验和
uint8_t *sp = frame.buffer;
uint8_t *ep = frame.buffer + offsetof(ros_send_data_frame_t, checksum);
for (uint8_t *p = sp; p < ep; ++p) {
    frame.checksum ^= *p;
}

// 发送数据
sendto(sock, frame.buffer, sizeof(frame), 0, 
       (struct sockaddr *)&target_addr, sizeof(target_addr));
```

### 4.2 接收控制命令

```c
uint8_t buffer[256];
ssize_t recvlen = recv(sock, buffer, sizeof(buffer), 0);

if (recvlen > 0 && ros_recv_data_format_check(buffer)) {
    ros_recv_data_frame_t *frame = (ros_recv_data_frame_t *)buffer;
  
    // 提取速度命令
    float linear_x = frame->velocity.x / 1000.0f;  // 转换为m/s
    float angular_z = frame->velocity.z / 1000.0f; // 转换为rad/s
  
    // 转换为左右轮速度
    float left_rpm = (linear_x - angular_z * wheel_space / 2) / wheel_perimeter;
    float right_rpm = (linear_x + angular_z * wheel_space / 2) / wheel_perimeter;
  
    // 设置电机速度
    mc_set_expect_rpm(left_motor, left_rpm);
    mc_set_expect_rpm(right_motor, right_rpm);
}
```

---

## 8. 错误处理与可靠性

### 8.1 ESP32端错误处理

#### 8.1.1 连接超时处理

```c
// 设置socket接收超时
struct timeval timeout = { .tv_sec = 5, .tv_usec = 0 };
setsockopt(g_chassis_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

// TCP连接超时处理
const struct timeval timeout = {.tv_sec = 10, .tv_usec = 0};
setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
```

#### 8.1.2 互斥锁保护机制

```c
// 发送数据时的互斥保护
if (xSemaphoreTake(g_lidar_send_muetx, 0)) {
    sendto(sock, buffer, len, 0,
        (struct sockaddr *)&g_lidar_target_addr,
        sizeof(g_lidar_target_addr));
    xSemaphoreGive(g_lidar_send_muetx);
}
```

#### 8.1.3 资源管理和清理

```c
// 统一的资源清理函数
esp_err_t stop_wireless_conn(void) {
    task_del();     // 删除所有任务
    sockfd_del();   // 关闭所有socket
    mutex_del();    // 删除所有互斥锁
    return ESP_OK;
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
```

### 9.2 性能优化策略

#### 9.2.1 网络层优化

```c
// 使用TCP_NODELAY减少TCP延迟
int flag = 1;
setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

// 调整socket缓冲区大小
int buffer_size = 8192;
setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
```

#### 9.2.2 任务优先级优化

```c
// 高优先级给实时数据传输任务
xTaskCreatePinnedToCore(lidar_send_task, "lidar_send", 4096,
    (void *)g_lidar_sock, 5, &g_task_handles[3], 1);  // 优先级5

// 中等优先级给配置管理任务
xTaskCreatePinnedToCore(conn_task, "conn", 4096,
    (void *)g_conn_sock, 3, &g_task_handles[0], 1);   // 优先级3
```

#### 9.2.3 数据传输优化

```python
# 批量数据传输，减少系统调用
def serial_to_socket(self):
    message = b""
    while self.running:
        # 累积数据后批量发送
        message += tmp if (tmp := self.seri.read_all()) is not None else b""
        if len(message) >= BATCH_SIZE or timeout_reached:
            sock.sendto(message, self.last_addr)
            message = b""
```

---

#### 10.3.2 监控工具

```bash
# 网络流量监控
sudo tcpdump -i wlan0 host micu-ros-car.local

# 虚拟串口数据监控
cat /tmp/pts/pts1  # 监控ROS端数据

# 系统资源监控
top -p $(pgrep python3)  # 监控Python进程
```

---

### 系统配置

1. **多字节数据**：采用小端序(Little Endian)
2. **数据对齐**：使用1字节对齐，避免填充字节
3. **单位标准**：速度单位为m/s，角速度单位为rad/s
