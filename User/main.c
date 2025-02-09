#include "Header.h"

volatile u8 delay_50,delay_flag; 		//��ʱ����			
u16 Voltage;							//��ѹ�������Ŵ�100������
u8 Flag_Stop;							//�����ͣ��־λ
u8 PS2_ON_Flag = 0,APP_ON_Flag = 0,ROS_ON_Flag = 0;		//Ĭ�����з�ʽ������
u8 Car_Num;								//���ͺ���ѡ��
u8 Flag_Show = 1;						//��ʾ��־λ��Ĭ�Ͽ����������л�����λ��ģʽ����ʱ�ر�
float Perimeter; 						//���ӵ��ܳ�
float Wheelspacing; 					//���ӵ��־�
u16 DISTANCE=0,ANGLE=0;
u8 one_lap_data_success_flag=0;         //�״��������һȦ�Ľ��ձ�־λ
int lap_count=0;//��ǰ�״���һȦ�����ж��ٸ���
int PointDataProcess_count=0,test_once_flag=0,Dividing_point=0;//�״�������ݵ�ļ���ֵ�����յ�һȦ�������һ֡���ݵı�־λ����Ҫ�и����ݵ�������
/**************************************************************************  
Function: Main function
Input   : none
Output  : none
�������ܣ�������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
int main(void)
{	
	u32 Voltage_Sum = 0;
	u8 Voltage_Count = 0;				//��ѹ������ر���
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�ж����ȼ�����
	LED_GPIO_Config();					//LED��ʼ����PC2��PC3
	Key_GPIO_Config();					//������ʼ����PA0��PC13  ������ʹ���״�
	BEEP_GPIO_Config();					//��������ʼ����PA15
	OLED_Init();						//OLED��ʼ��
	Encoder_Init();						//��������ʼ����TIM4��TIM8 
	Motor_Init(7199,0);					//���PWM��ʼ����TIM3
	DEBUG_USART_Init();					//���Դ������ã�����1��������115200
	BLUETOOTH_USART_Init();				//�����������ã�����3��������9600
	LIDAR_USART_Init();					//�״ﴮ�����ã�����5��������115200
  
  
  //����2
  UART_Init();
//  USART_Send('1');
  
  
	PS2_Init();							//��ʼ��PS2�ֱ��ӿ�
	PS2_SetInit();						//PS2�ֱ�����Ϊģ����ģʽ
	Car_Select_ADC_Init();				//����ѡ��ADC��ʼ��
	Voltage_ADC_Init();					//���ڲ�����ѹ
//	Distance_Cap_Init(0XFFFF,72-1);		//���ڲ����������ľ��룬Ĭ��ע�ͣ����忴capture.h
//	PWM_Cap_Init(0XFFFF,72-1);			//����4·��ģң�س�ʼ����Ĭ��ע�ͣ����忴capture.h   /*�������ͺ�ģֻ��ʹ����һ*/
	MPU6050_Init();										//MPU6050��ʼ��
	Robot_Select();
	BEEP_ON;													//�������·�������ʾ
	delay_ms(200);
	BEEP_OFF;
    if(Car_Num == Akm_Car)
		Servo_Init(9999,71);									//���������ͳ�ʼ�����
    Servo_Init3(9999,71);
    
	Car_Perimeter_Init();										//��ʼ�������ܳ����־�
	TIMING_TIM_Init(7199,49);									//5ms�жϿ��ƣ��󲿷ֿ����߼������� ->��ذ�����control �״�-���ܿ��� ������5ms����һ������
	
//   while (1) {
//      if (data_ready) {  // ��������ѽ���
//          data_ready = 0;  // ���ñ�־λ
//          USART_SendString((const char*)rx_buf);  // ���ͽ��յ�������
//          USART_Send('\n');  // ��ӻ��з�
//      }
//    }
    uint8_t processing_buf[RX_BUF_SIZE];
    
   while(1)
	{
//      if (data_ready) {
//            memcpy(processing_buf, rx_buf, RX_BUF_SIZE);
//            data_ready = 0;
//            parse_received_data(processing_buf);
//        }
//      USART_SendString((const char*)rx_buf);  // ���ͽ��յ�������
//    if (data_ready) {  // ��������ѽ���
//          data_ready = 0;  // ���ñ�־λ
//          USART_SendString((const char*)rx_buf);  // ���ͽ��յ�������
//          USART_Send('\n');  // ��ӻ��з�
//      }
//    
      ProcessReceivedData();
    
		Robot_Select();
		if(Flag_Show)											//������ʾ������λ��ģʽ
		{
			LED2_OFF;
			PS2_Read();											//�ֱ����ݶ�ȡ
			Show();												//��ʾ��
           APP_Show();											//�ֻ�������ʾ
			
			Voltage_Sum += Get_Voltage();						//��ѹ������ÿ50��ȡһ��ƽ��
      
			if((++Voltage_Count) == 5)
				Voltage = Voltage_Sum/5,Voltage_Count = 0,Voltage_Sum = 0;
		}
		else													//������λ��ģʽ
		{
			LED2_ON;											//LED2����ָʾ��������λ��ģʽ
			DataScope();										//��λ��ʾ������ʾ
		}
		delay_flag=1;	//ʹ��50ms��ʱʱ���״����ݻ�����쳣
		delay_50=0;
		while(delay_flag);	     								//ͨ����ʱ�ж�ʵ�ֵ�50ms��ʱ����Ҫ����ʾ����	
      //������Ļˢ�µĹ���  ��Ҫ����ʾ����	һ����20��ÿ��
	}
}


/***********************************END OF FILE********************************/

