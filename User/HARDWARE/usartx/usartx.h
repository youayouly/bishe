/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2023-03-02

All rights reserved
***********************************************/
#ifndef __USRATX_H
#define __USRATX_H 

#include "Header.h"


#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

//����2-USART2
//ROS����

#define  ROS_USARTx                   USART1
#define  ROS_USART_CLK                RCC_APB2Periph_USART1
#define  ROS_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  ROS_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  ROS_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  ROS_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd

#define  ROS_USART_TX_GPIO_PORT       GPIOA   
#define  ROS_USART_TX_GPIO_PIN        GPIO_Pin_2
#define  ROS_USART_RX_GPIO_PORT       GPIOA
#define  ROS_USART_RX_GPIO_PIN        GPIO_Pin_3

#define  ROS_USART_IRQ                USART2_IRQn
#define  ROS_USART_IRQHandler         USART2_IRQHandler

//���������͵���Сת��뾶���ɻ�е�ṹ�������־ࡢ��ࡢǰ�����ת��
#define MINI_AKM_MIN_TURN_RADIUS 0.350f 

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****���ڴ�������Ǽ��ٶȼ��������ݵĽṹ��*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2���ֽ�
	short Y_data; //2 bytes //2���ֽ�
	short Z_data; //2 bytes //2���ֽ�
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******���ڷ������ݵĽṹ��*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1���ֽ�
		short X_speed;	            //2 bytes //2���ֽ�
		short Y_speed;              //2 bytes //2���ֽ�
		short Z_speed;              //2 bytes //2���ֽ�
		short Power_Voltage;        //2 bytes //2���ֽ�
		Mpu6050_Data Accelerometer; //6 bytes //6���ֽ�
		Mpu6050_Data Gyroscope;     //6 bytes //6���ֽ�	
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		float X_speed;	            //4 bytes //4���ֽ�
		float Y_speed;              //4 bytes //4���ֽ�
		float Z_speed;              //4 bytes //4���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Control_Str;
}RECEIVE_DATA;


void data_transition(void);
void USART1_SEND(void);
float float_abs(float insert);

int USART1_IRQHandler(void);

float Vz_to_Akm_Angle(float Vx, float Vz);
float XYZ_Target_Speed_transition(u8 High,u8 Low);

void usart1_send(u8 data);


u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);


#endif
