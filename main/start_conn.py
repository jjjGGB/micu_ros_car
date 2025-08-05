#!/usr/bin/env python3
from threading import Thread
from pathlib import Path
import subprocess as sp
import socket
import serial
import time
import json
import re
import os
from pathlib import Path

chassis_config_path = Path(__file__).parent / "src/chassis_drive/launch/chassis_start.launch"
lidar_config_path = Path(__file__).parent / "src/ydlidar_ros_driver/launch/X2.launch"
script_dir = os.path.dirname(os.path.abspath(__file__))
pts_dir = os.path.join(script_dir, 'pts') 
os.makedirs(pts_dir, exist_ok=True)  # 确保目录存在


with \
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s1, \
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s2:

    s1.bind((socket.gethostname(), 0))
    s2.bind((socket.gethostname(), 0))
    s1.settimeout(5)
    s2.settimeout(5)

    serial_ports = {
        "lidar": (f"/{pts_dir}/pts0", f"/{pts_dir}/pts1"),
        "chassis": (f"/{pts_dir}/pts2", f"/{pts_dir}/pts3"),
    }

    #创建虚拟串口
    args = lambda item: ["socat", f"pty,raw,echo=0,link={item[0]}", f"pty,raw,echo=0,link={item[1]}"]
    vserial_popens = [sp.Popen(args(item), stdout=sp.DEVNULL, stderr=sp.DEVNULL) for item in serial_ports.values()]

    while all((Path(path).exists() for pair in serial_ports.values() for path in pair)):
        if None not in [popen.poll() for popen in vserial_popens]:
            raise RuntimeError("create socat vitual serial failed")
   
    #替换配置文件中的串口参数
    pattern = r'(<\s*param\s+name="usart_port_name"\s+type="string"\s+value=")([^"]*)("\s*/>)'
    repl = rf'\g<1>{serial_ports["chassis"][1]}\g<3>'
    with open(chassis_config_path) as f:
        s = f.read()
    with open(chassis_config_path, "w") as f:
        f.write(re.sub(pattern, repl, s))

    pattern = r'(<\s*param\s+name="port"\s+type="string"\s+value=")([^"]*)("\s*/>)'
    repl = rf'\g<1>{serial_ports["lidar"][1]}\g<3>'
    with open(lidar_config_path) as f:
        s = f.read()
    with open(lidar_config_path, "w") as f:
        f.write(re.sub(pattern, repl, s))

    def connect_esp32_car(local_port, toaddr,device_type):
        message = json.dumps({
        "port": local_port, 
        "type": device_type  # "chassis" 或 "lidar"
    	})
        with socket.socket() as s:  #创建TCP客户端socket
            try:
                s.connect(toaddr)# TCP连接到ESP32的TCP服务器
                s.send(message.encode("ascii"))#发送JSON消息
                print(f"Sending JSON config: {message}")
                response = s.recv(1024)
                print(f"Received response from ESP32: {response.decode()}")
            except socket.error as e:
                raise RuntimeError(f"cannot found the dns or ip address: {toaddr[0]}") from e

    class Communication:
        def __init__(self, serial_port: str, tcpaddr, device_type: str) -> None:
            self.seri = serial.Serial(serial_port, timeout=0.01)
            self.tcpaddr = tcpaddr
            self.device_type = device_type
            self.last_addr = None

        def start(self):
            self.running = True
            ths = [Thread(target=self.socket_to_serial), Thread(target=self.serial_to_socket)]
            [th.start() for th in ths]
            return ths
        
        def stop(self):
            self.running = False

        def __enter__(self):
            return self.start()

        def __exit__(self, *_):
            self.stop()

        def socket_to_serial(self):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                # 1. 创建UDP套接字并绑定到随机端口
                sock.bind(('0.0.0.0', 0))
                port = sock.getsockname()[1]
                print(f"UDP socket bound to port: {port}")
                sock.settimeout(5)
                # 2. 通过TCP连接告诉ESP32这个UDP端口
                connect_esp32_car(port, self.tcpaddr, self.device_type)
                print(f"TCP connection established with {self.tcpaddr}")
                while self.running:
                    try:
                    	# 接收ESP32发来的数据，并写入串口
                        message, self.last_addr = sock.recvfrom(1024)                        
                        self.seri.write(message)
                        print(self.tcpaddr, self.device_type,len(message), hex(message[0]))
                    except socket.timeout:
                        connect_esp32_car(port, self.tcpaddr, self.device_type)
                        print(f"reconnect smart car: {self.tcpaddr}")

        def serial_to_socket(self):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                message = b""
                while self.running:
                    #读取虚拟串口的数据，通过 UDP 发送到 ESP32。
                    message += tmp if (tmp := self.seri.read_all()) is not None else b""
                    if self.last_addr is not None and len(message) > 0:
                        print(f"[{self.device_type}] Read from serial: {message.hex()}")
                        sock.sendto(message, self.last_addr)
                        print(f"[{self.device_type}] Sent to ESP32: {message.hex()}")
                        message = b""

    time.sleep(1)

    comms = {
        "lidar": Communication(
            serial_ports["lidar"][0],
            ("micu-ros-car.local", 8080),
            "lidar",
        ),
        "chassis": Communication(
            serial_ports["chassis"][0],
            ("micu-ros-car.local", 8080),
            "chassis"
        )
    }

    thread_groups = {hostname: c.start() for hostname, c in comms.items()}

    for hostname, c in comms.items():
        print(f"{hostname} serial: {serial_ports[hostname][0]}")
    
    while True:
        for hostname, group in thread_groups.items():
            if not all(map(lambda x: x.is_alive(), group)):
                [c.stop() for c in comms.values()]
                raise RuntimeError(f"{hostname} pipe ended abnormally")
