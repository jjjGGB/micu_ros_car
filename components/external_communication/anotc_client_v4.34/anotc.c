#include "anotc.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

uint8_t data_to_send[50];	//发送数据缓存
int _temp1;


void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)  //发送校验数据   head 帧头    check_sum  接收到的数据生成的校验
{
	uint8_t sum = 0,i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;

	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	UDP_write_anotc(data_to_send, 7);
}




void ANO_DT_Send_USER_DATA(char command, uint16_t data)
{
	uint8_t _cnt=0; //发送缓冲区字节数
	uint8_t sum = 0;//校验
	uint16_t _temp;//临时变量1

	data_to_send[_cnt++]=0xAA;//4个A的帧头
	data_to_send[_cnt++]=0xAA;//4个A的帧头
	data_to_send[_cnt++]=command;//功能字为05
	data_to_send[_cnt++]=0;	  //这个是数据长度现在先不填后面再填

	data_to_send[_cnt++]=BYTE1(data);//int16高位填充
	data_to_send[_cnt++]=BYTE0(data);//int16低位填充

	data_to_send[3] = _cnt-4; //_cnt累计的长度减去前面的4个非数据字节等于数据的长度

	for(int i = 0 ; i < _cnt ; i++) sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	UDP_write_anotc(data_to_send, _cnt);
}


//向匿名上位机发送PID数据
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0,sum = 0,i;
	uint16_t _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;  // 设置为不同的PID帧
	data_to_send[_cnt++]=0;


	_temp = p1_p * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	UDP_write_anotc(data_to_send, _cnt);
}

//向上位机发送传感器原始数据
void ANO_DT_Send_Senser(uint16_t a_x,uint16_t a_y,uint16_t a_z,uint16_t g_x,uint16_t g_y,uint16_t g_z,uint16_t m_x,uint16_t m_y,uint16_t m_z,uint32_t bar)
{
	uint8_t _cnt=0;
	uint16_t _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;

	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	UDP_write_anotc(data_to_send, _cnt);

}

void anotc_send(void * pvParameters)
{//匿名上位机数据发送
	static uint16_t slow_down = 0;

	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 1;//1ms运行一次
	while(1)
	{
		vTaskDelayUntil(&adp,adg);


		if(++slow_down==500) slow_down=0;

		if(slow_down%49==0)//刷新速度状态
		{

			ANO_DT_Send_USER_DATA(0xF1, current_speeds[0]);
			ANO_DT_Send_USER_DATA(0xF2, current_speeds[1]);
			ANO_DT_Send_USER_DATA(0xF3, target_speeds[0]);
			ANO_DT_Send_USER_DATA(0xF4, target_speeds[1]);

		}
		if(slow_down%109==0)//刷新姿态状态
		{
			ANO_DT_Send_USER_DATA(0xF5, Send_Data.Sensor_Str.X_speed);
			ANO_DT_Send_USER_DATA(0xF6, Receive_Data.Control_Str.X_speed);
			ANO_DT_Send_USER_DATA(0xF7, proto_imu_data.euler[0]);//roll2

		}
		if(slow_down%209==0)//刷新姿态状态
		{
			ANO_DT_Send_USER_DATA(0xF8, Send_Data.Sensor_Str.Z_speed);
			ANO_DT_Send_USER_DATA(0xF9, Receive_Data.Control_Str.Z_speed);
			ANO_DT_Send_USER_DATA(0xFA, proto_imu_data.euler[1]);//pitch2
		}
		// if(slow_down%309==0)//刷新姿态状态
		// {
		// 	ANO_DT_Send_USER_DATA(0xF, Send_Data.Sensor_Str.Z_speed);
		// 	ANO_DT_Send_USER_DATA(0xF9, Receive_Data.Control_Str.Z_speed);
		// 	ANO_DT_Send_USER_DATA(0xFA, proto_imu_data.euler[2]);//yaw
		// }

		
	}
}


void anotc_task(void)//匿名上位机初始化
{

	//PidParameter_init();

	xTaskCreate(anotc_send, "anotc_send",1024*5,NULL,4,NULL);

}
