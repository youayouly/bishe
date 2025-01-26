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

#include "Lidar.h"
#include <string.h>

PointDataProcessDef PointDataProcess[420];//����390������
PointDataProcessDef Dataprocess[400];      //����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����
PointDataProcessDef TempData[12]={0};  //������0�ȵ���һȦ������ʱ�洢
LiDARFrameTypeDef Pack_Data;			//�״���յ����ݴ������������֮��

static const uint8_t CrcTable[256] =
{
 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};//����crcУ�������



/**************************************************************************
Function: LIDAR_USART_IRQHandler
Input   : none
Output  : none
�������ܣ��״�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
//����5
void LIDAR_USART_IRQHandler(void)//����ld14�״����ݣ�һ֡47���ֽ�
{	
	static u8 state = 0;//״̬λ	��ָʾ��ǰ����֡��λ��
	static u8 crc = 0;	//У���
	static u8 cnt = 0;	//����һ֡12����ļ���
	
	u8 temp_data;
	if(USART_GetITStatus(LIDAR_USARTx, USART_IT_RXNE) != RESET) //���յ�����
	{	
    USART_ClearITPendingBit(LIDAR_USARTx,USART_IT_RXNE);		
		temp_data=USART_ReceiveData(LIDAR_USARTx); 
#ifdef LD14	
		if (state > 5)	
		{
			if(state < 42)
			{
				if(state%3 == 0)		//һ֡�����е����Ϊ6,9.....39�����ݣ�����ֵ��8λ
				{
					Pack_Data.point[cnt].distance = (u16)temp_data;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(state%3 == 1)	//һ֡�����е����Ϊ7,10.....40�����ݣ�����ֵ��8λ
				{
					Pack_Data.point[cnt].distance = ((u16)temp_data<<8)+Pack_Data.point[cnt].distance;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else					//һ֡�����е����Ϊ8,11.....41�����ݣ����Ŷ�
				{
					Pack_Data.point[cnt].confidence = temp_data;
					cnt++;	
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(state)
				{
					case 42:
						Pack_Data.end_angle = (u16)temp_data;						//�����Ƕȵ�8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						Pack_Data.end_angle = ((u16)temp_data<<8)+Pack_Data.end_angle;//�����Ƕȸ�8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						Pack_Data.timestamp = (u16)temp_data;						//ʱ�����8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						Pack_Data.timestamp = ((u16)temp_data<<8)+Pack_Data.timestamp;//ʱ�����8λ
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						Pack_Data.crc8 = temp_data;		//�״ﴫ����У���
						if(Pack_Data.crc8 == crc)		//У����ȷ
						{
							data_process();				//���յ�һ֡��У����ȷ���Խ������ݴ���
						}
						else
						{
							memset(&Pack_Data, 0, sizeof(LiDARFrameTypeDef));//����
						}
						crc = 0;
						state = 0;
						cnt = 0;//��λ
						break;
					default: break;
				}
			}
		}
		else 
		{
			switch(state)
			{
				case 0:
					if(temp_data == HEADER)									//ͷ�̶�
					{
						Pack_Data.header = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];				//��ʼ����У��
					} else state = 0,crc = 0;
					break;
				case 1:
					if(temp_data == LENGTH)									//�����ĵ�����Ŀǰ�̶�
					{
						Pack_Data.ver_len = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else state = 0,crc = 0;
					break;
				case 2:
					Pack_Data.speed = (u16)temp_data;						//�״��ת�ٵ�8λ����λ��ÿ��
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					Pack_Data.speed = ((u16)temp_data<<8)+Pack_Data.speed;	//�״��ת�ٸ�8λ
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					Pack_Data.start_angle = (u16)temp_data;					//��ʼ�Ƕȵ�8λ���Ŵ���100��
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					Pack_Data.start_angle = ((u16)temp_data<<8)+Pack_Data.start_angle;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;
			}
		}
#endif
	}	
} 
/**************************************************************************
Function: data_process
Input   : none
Output  : none
�������ܣ����ݴ�����
��ڲ�������
����  ֵ����
**************************************************************************/
//���һ֡���պ���д���
void data_process(void)
{
	//�״�ÿ֡�����ǹ̶�12���㣬�״�һȦ��Լ��xx���㵫�ǲ���һ���̶�ֵ���״�һ��ת��ԼxxȦ��
	//�����ǵ��״�ת��1Ȧ���ٽ�һ��Ȧ��������ʹ�ö�����һ֡һ֡���ݵ�ʵʱ���¶��ǻ�ȡ��һ��Ȧ�״����ݺ��ٸ��¡�
	//�����״�ÿһȦ�ĵ��������ǹ̶��ģ���Ϊת�ٲ��Ǻ㶨����΢΢Сƫ�������ÿһȦ�ĵ�����һ֡���ݺܴ�����ǡ�ǰ��������ǵ�ǰ��һȦ��ĩβ��������������һȦ�Ŀ�ͷ����
	//����״����ݴ����ʱ��������һ֡���ݵĿ�ʼ�ǶȺͽ����Ƕ��ж��Ƿ�����ݽ����и
	u16 i,j,m;
	float start_angle = Pack_Data.start_angle/100.0;  //����12����Ŀ�ʼ�Ƕȣ����ִ���ʱ�Ŵ���100��
	float end_angle = Pack_Data.end_angle/100.0;
	float area_angle[12]={0};//һ֡���ݵ�ƽ���Ƕ�	
	//180�ȵĵط��ǳ�ͷ����һ����ת��ʹ��ͷ������0��
	if((start_angle -= 180)<0)
		start_angle += 360;
	if((end_angle -= 180)<0)
		end_angle += 360;
	if(start_angle>end_angle)//��ʼ�ͽ����Ƕȱ�0�ȸպ÷ֿ������
	{
	   end_angle +=360;	
	}
	for(m=0;m<12;m++) //��ȡÿ����ĽǶ�ֵ
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/10.5*m;
		if(area_angle[m]>360.0)  //���1����ʼ�ͽ����Ƕȱ�0�ȸպ÷ֿ������
		{
			if(test_once_flag==0) //�ҵ��ڼ������Ƿֽ��
		  {
		    test_once_flag=1; //�ҵ��˱�־λ��1
		    Dividing_point=m;  	
			area_angle[m] -=360; //��ԭ�����ȷ�Ƕ�ֵ
		   }
		  else area_angle[m] -=360; //��ԭ�����ȷ�Ƕ�ֵ
		}
	}
	if ((end_angle <= 360) && (end_angle >= 359) ) //���2������������պ����ݲ���0���и0���Ǳ���֡���ݶ������м�
	{
	    test_once_flag=1;
		Dividing_point=12;
	  }
	if(test_once_flag) //��֡����Ҫ�������и���������֡���ݰ�������һȦ�����ݺ���һȦ�����ݣ�
	{
		
		 for(i=0;i<Dividing_point;i++)    //��һ֡����360��֮ǰ�ĵ���������
		 {
			  PointDataProcess[PointDataProcess_count+i].angle = area_angle[i];
			  PointDataProcess[PointDataProcess_count+i].distance = Pack_Data.point[i].distance;
		 }
		 PointDataProcess_count=PointDataProcess_count+Dividing_point;
		  for(j=0;j<12-Dividing_point;j++)    //����һȦ���ݴ������ʱ�洢������
		 {
			TempData[j].angle = area_angle[j+Dividing_point];
			TempData[j].distance = Pack_Data.point[j+Dividing_point].distance;
		 }
		 for(m=0;m<PointDataProcess_count;m++)    //��ʱһ��Ȧ�������Ѿ��ռ���ɣ���һ��Ȧ���ݸ��Ƶ���һ���������ʹ��
		 {
			 Dataprocess[m].angle=PointDataProcess[m].angle;
			 Dataprocess[m].distance = PointDataProcess[m].distance;
		 }
		 lap_count=PointDataProcess_count;//��ȡ��ǰ����һȦ�ж��ٸ���
		 one_lap_data_success_flag=1;//һȦ���ݸ������
		 test_once_flag=0; //��־λ���㣬��һ֡���ݱض�����Ҫ�������и�
		 PointDataProcess_count=0;//����ڼ�����ļ����������
	}
 else//�������ݲ���Ҫ�������и�
 {
	  	for(i=0;i<12;i++)    //��һ֡���ݴ����distance_sum������
	    {
			if(one_lap_data_success_flag) //�����һ֡���������и��ô��Ҫ�Ȱ�֮ǰ��ʱ�洢������ȡ����������һȦ���ݵ���ǰ��
			 {
				 for(j=0;j<12-Dividing_point;j++)    
				 {
					PointDataProcess[j].angle = TempData[j].angle;
					PointDataProcess[j].distance = TempData[j].distance; 
				 }
				one_lap_data_success_flag=0;//�������µ�һȦ���ݣ�����
				PointDataProcess_count=PointDataProcess_count+(12-Dividing_point);//�ۼ�
				Dividing_point=0;//�ж��и���������
			 }
     	  PointDataProcess[PointDataProcess_count+i].angle = area_angle[i];
		  PointDataProcess[PointDataProcess_count+i].distance = Pack_Data.point[i].distance;
	   }
	   PointDataProcess_count=PointDataProcess_count+12; //�ۼ�12
 }
}

