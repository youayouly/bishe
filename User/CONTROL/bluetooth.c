#include "bluetooth.h"  
u8 PID_Send;			//�����ֻ�app��ȡ��������ı���
u8 Flag_Direction;		//�����ֻ�app�ķ��������˳ʱ��һȦ��8��������ֵ��1--8,ֹͣʱ��ֵΪ0
/**************************************************************************
Function: BLUETOOTH_USART_IRQHandler
Input   : none
Output  : none
�������ܣ����������жϺ��������������ֻ�������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//���������жϣ�����3
void BLUETOOTH_USART_IRQHandler(void)
{
	static int temp_count = 0;				//���ڼ�¼ǰ����ָ��Ĵ�������һ������������ʱ����Ҫ�õ�
	static	int bluetooth_receive=0;		//����������ر���
	static u8 Flag_PID,i,j,Receive[50];
	static float Data;						//app������ղ����õ��ı���
	if(USART_GetITStatus(BLUETOOTH_USARTx, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		bluetooth_receive=USART_ReceiveData(BLUETOOTH_USARTx); 
		USART_ClearITPendingBit(BLUETOOTH_USARTx,USART_IT_RXNE);
		if(APP_ON_Flag == RC_OFF)						//δ�����������ƣ�ֻ�������ݽ��м򵥵ķ���
		{
			if(bluetooth_receive == 0x41)
			{
				if((++temp_count) == 5)					//��Ҫ��������5��ǰ����ָ�����ת��һ��ʱ��ɿ�ʼapp����
				{
					temp_count = 0;
					APP_ON_Flag = RC_ON;		
					PS2_ON_Flag = RC_OFF;
					ROS_ON_Flag=RC_OFF;
					RC_Velocity = Default_Velocity;		//��ʱ�ٶȻָ�Ĭ���ٶ�
					if(Car_Num == Akm_Car)				//����������ת��Ϊ�Ƕȣ���������ʱת��Ƕ�Ĭ�ϲ���
						RC_Turn_Velocity = Bluetooth_Turn_Angle;
					else								//�������������ֲ���
						RC_Turn_Velocity = Default_Turn_Bias;
				}
			}
			else 
				temp_count = 0;
		}
		else 
		{
			if(bluetooth_receive>=0x41&&bluetooth_receive<=0x48) 	//Ĭ��ʹ�ã�8�������ֻ����͵���ֵ��0x41--0x48
				Flag_Direction=bluetooth_receive-0x40;
			
			else if(bluetooth_receive==0X5A)  						//ֹͣʱ����0x5a
				Flag_Direction=0;

			else if(bluetooth_receive<=10)  //����app
				Flag_Direction=bluetooth_receive;	
			
			else if(bluetooth_receive==0x59) 						//���ٰ���
			{
				if((RC_Velocity -= X_Step)<MINI_RC_Velocity)
					RC_Velocity = MINI_RC_Velocity;
				if(Car_Num != Akm_Car)								//�ǰ�������ת���ٶȿɸ���
				{
					if((RC_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)
					RC_Turn_Velocity = MINI_RC_Turn_Velocity;
				}
			}			 
			else if(bluetooth_receive==0x58)						//����
			{
				if((RC_Velocity += X_Step)>MAX_RC_Velocity)
					RC_Velocity = MAX_RC_Velocity;
				if(Car_Num != Akm_Car)
				{
					if((RC_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)
						RC_Turn_Velocity = MAX_RC_Turn_Bias;
				}
			}			
			else if(bluetooth_receive==0x7B) Flag_PID=1;   			//APP����ָ����ʼλ
			else if(bluetooth_receive==0x7D) Flag_PID=2;   			//APP����ָ��ֹͣλ

			if(Flag_PID==1)  //�ɼ�����
			{
				Receive[i]=bluetooth_receive;
				i++;
			}
			if(Flag_PID==2)  //��������
			{
				if(Receive[3]==0x50) 	 PID_Send=1;
				else if(Receive[1]!=0x23) 
				{								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
								switch(Receive[1])//���ν���
								{  
								case 0x30:  RC_Velocity=Data;break;
								case 0x31:  CCD_Move_X=Data/1000;break;//mm/s��Ϊm/s
								case 0x32:  break;
								case 0x33:  break; //Ԥ��
								case 0x34:  break; //Ԥ�� 
								case 0x35:  break; //Ԥ��
								case 0x36:  break; //Ԥ��
								case 0x37:  break; //Ԥ��
								case 0x38:  break; //Ԥ��
								default:break;
								}
				}				 
				Flag_PID=0;
				i=0;
				j=0;
				Data=0;
				memset(Receive, 0, sizeof(u8)*50);//��������
			} 
		}
	}  											 
}



