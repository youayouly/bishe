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

#ifndef __LIDAR_H
#define	__LIDAR_H

#include "stm32f10x.h"
#include "Header.h"


//��ѡ�״�
//Ŀǰֻ��LD14
#define 	LD14		
//#define 	LD06
//#define 	N10
//#define 	M10


#ifdef LD14

#define ANGLE_PER_FRAME 				12
#define HEADER 							0x54
#define POINT_PER_PACK 					12
#define LENGTH  						0x2C 	//����λ��һ֡���ݽ��յ��ĵ�����Ŀǰ�̶���12������λԤ��


#define offset_x						5.9f
#define offset_y						-20.14f

#endif


typedef struct __attribute__((packed)) Point_Data
{
	u16 distance;//����
	u8 confidence;//���Ŷ�
	
}LidarPointStructDef;//һ֡������ÿ�������������


typedef struct __attribute__((packed)) Pack_Data
{
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
}LiDARFrameTypeDef;//һ֡���ݽṹ��

typedef struct __attribute__((packed)) PointDataProcess_
{
	u16 distance;
	float angle;
}PointDataProcessDef;//��������������

extern PointDataProcessDef PointDataProcess[420];//����390������
extern PointDataProcessDef Dataprocess[400];//����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef TempData[12];  //������0�ȵ���һȦ������ʱ�洢

void data_process(void);
#endif


