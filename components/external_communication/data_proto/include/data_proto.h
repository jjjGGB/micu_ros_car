#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
		int16_t X;
		int16_t Y;
		int16_t Z;
} INT16_XYZ;

typedef struct {
		float X;
		float Y;
		float Z;
} FLOAT_XYZ;

#define FRAME_HEADER      0X7B  //帧头
#define FRAME_TAIL        0X7D  //帧尾
#define SEND_DATA_SIZE    24    //ros发送数据长度
#define RECEIVE_DATA_SIZE 11    //ros接收数据长度

typedef struct __Mpu6050_Data_  //用于存放陀螺仪加速度计三轴数据的结构体
{
	int16_t X_data; //2字节 x轴
	int16_t Y_data; //2字节 y轴
	int16_t Z_data; //2字节 z轴
}Mpu6050_Data;

typedef struct _SEND_DATA_ //发送给ROS系统反馈数据的结构体
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1字节 帧头
		int16_t X_speed;	            //2字节 x轴速度
		int16_t Y_speed;              //2字节 y轴速度
		int16_t Z_speed;              //2字节 z轴速度
		int16_t Power_Voltage;        //2字节 电池电压
		Mpu6050_Data Accelerometer; //6字节 三轴加速度
		Mpu6050_Data Gyroscope;     //6字节 三轴陀螺仪
		unsigned char Frame_Tail;   //1字节 帧尾
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_ //接收ros系统的数据结构体
{
	unsigned char buffer[10];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1字节 帧头
		float X_speed;	            //4字节 x轴速度
		float Y_speed;              //4字节 y轴速度
		float Z_speed;              //4字节 z轴速度
		unsigned char Frame_Tail;   //1字节 帧尾
	}Control_Str;
}RECEIVE_DATA;






#ifdef __cplusplus
}
#endif
