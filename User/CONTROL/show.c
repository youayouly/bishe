#include "show.h"

/**************************************************************************
Function: OLED_Show
Input   : none
Output  : none
�������ܣ���ʾ����ʾ����
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void Show(void)
{
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM���㵫������ˢ�£���ֹ����
	if(Mode==ELE_Line_Patrol_Mode)			//���Ѳ��ģʽ��ʾ
	{
		//=============��һ����ʾС��ģʽ=======================//	
		OLED_ShowNumber(0,0,Sensor_Left,4,12);	
		OLED_ShowNumber(30,0,Sensor_Middle,4,12);
		OLED_ShowNumber(60,0,Sensor_Right,4,12);
		OLED_ShowNumber(90,0,Sensor,4,12);		
		//=============�ڶ�����ʾX��Z����ٶ�===============//	
		OLED_ShowString(00,10,"MoveX:");
		if(Move_X<0)		OLED_ShowString(50,10,"-");
		else				OLED_ShowString(50,10,"+");
							OLED_ShowNumber(60,10,myabs(Move_X),3,12);
		
		if( Move_Z<0)		OLED_ShowString(94,10,"-");
		else				OLED_ShowString(94,10,"+");
							OLED_ShowNumber(106,10,myabs(Move_Z),3,12);

		//=============��������ʾ����PWM���ٶ�����=======================//		
								OLED_ShowString(00,20,"L-P:");
		if(MotorA.Motor_Pwm<0)	OLED_ShowString(30,20,"-"),
								OLED_ShowNumber(36,20,MotorA.Motor_Pwm,4,12);
		if(MotorA.Motor_Pwm>=0)	OLED_ShowString(30,20,"+"),
								OLED_ShowNumber(36,20,MotorA.Motor_Pwm,4,12);
										
								OLED_ShowString(66,20,"L-E:");

		if(MotorA.Target_Encoder<0)	  	OLED_ShowString(98,20,"-");
		if(MotorA.Target_Encoder>=0)  	OLED_ShowString(98,20,"+");
									OLED_ShowNumber(104,20,MotorA.Target_Encoder*1000,4,12);

		//=============��������ʾ�ҵ��PWM���ٶ�����=======================//		
								OLED_ShowString(00,30,"R-P:");
		if(MotorB.Motor_Pwm<0)	OLED_ShowString(30,30,"-"),
								OLED_ShowNumber(36,30,MotorB.Motor_Pwm,4,12);
		if(MotorB.Motor_Pwm>=0)	OLED_ShowString(30,30,"+"),
								OLED_ShowNumber(36,30,MotorB.Motor_Pwm,4,12);
										
								OLED_ShowString(66,30,"R-E:");

		if(MotorB.Target_Encoder<0)		OLED_ShowString(98,30,"-");
		if(MotorB.Target_Encoder>=0)  	OLED_ShowString(98,30,"+");
									OLED_ShowNumber(104,30,MotorB.Target_Encoder*1000,4,12);
		//=============��������ʾ�״��Ƿ����ϰ���=======================//
		OLED_ShowString(0,40,"Detect:");
		if(Lidar_Detect == Lidar_Detect_ON)
			OLED_ShowString(65,40,"ON ");
		else
			OLED_ShowString(65,40,"OFF");
		//=============��������ʾģʽ����ѹ��������=======================//
//		if(Car_Num == Akm_Car)				OLED_ShowString(0,50,"Akm  ");
//		else if(Car_Num == Diff_Car)		OLED_ShowString(0,50,"Diff ");
//		else if(Car_Num == Small_Tank_Car)	OLED_ShowString(0,50,"STank");
//		else 								OLED_ShowString(0,50,"BTank");
								OLED_ShowString(0,50,"ELE ");				//ģʽ��ʾ
								OLED_ShowString(54,50,".");
								OLED_ShowString(78,50,"V");
								OLED_ShowNumber(40,50,Voltage/100,2,12);
								OLED_ShowNumber(62,50,Voltage/10%10,1,12);//��ʾ��ѹ
								OLED_ShowNumber(70,50,Voltage%10,1,12);
		if(Flag_Stop)          	OLED_ShowString(105,50, "OFF");
		if(!Flag_Stop)        	OLED_ShowString(105,50, "ON ");

	}
	else if(Mode == CCD_Line_Patrol_Mode)				//CCDѲ��ģʽ��ʾ
	{
		OLED_Show_CCD();
		//=============�ڶ�����ʾCCD��ֵ=======================//	
								OLED_ShowString(00,10,"Median:");
		if( CCD_Median<0)		OLED_ShowString(90,10,"-"),
								OLED_ShowNumber(100,10,-CCD_Median,3,12);
		else                 	OLED_ShowString(90,10,"+"),
								OLED_ShowNumber(100,10, CCD_Median,3,12);

		
		//=============��������ʾCCD��ֵ=======================//	
								OLED_ShowString(00,20,"Threshold:");
		if( CCD_Threshold<0)	OLED_ShowString(90,20,"-"),
								OLED_ShowNumber(100,20,-CCD_Threshold,3,12);
		else                 	OLED_ShowString(90,20,"+"),
								OLED_ShowNumber(100,20, CCD_Threshold,3,12);

		//=============��������ʾ����PWM���ٶ�����=======================//		
								OLED_ShowString(00,30,"L-P:");
		if(MotorA.Motor_Pwm<0)	OLED_ShowString(30,30,"-"),
								OLED_ShowNumber(36,30,MotorA.Motor_Pwm,4,12);
		if(MotorA.Motor_Pwm>=0)	OLED_ShowString(30,30,"+"),
								OLED_ShowNumber(36,30,MotorA.Motor_Pwm,4,12);
										
								OLED_ShowString(66,30,"L-E:");

		if(MotorA.Target_Encoder<0)	  	OLED_ShowString(98,30,"-");
		if(MotorA.Target_Encoder>=0)  	OLED_ShowString(98,30,"+");
									OLED_ShowNumber(104,30,MotorA.Target_Encoder*1000,4,12);

		//=============��������ʾ�ҵ��PWM���ٶ�����=======================//		
								OLED_ShowString(00,40,"R-P:");
		if(MotorB.Motor_Pwm<0)	OLED_ShowString(30,40,"-"),
								OLED_ShowNumber(36,40,MotorB.Motor_Pwm,4,12);
		if(MotorB.Motor_Pwm>=0)	OLED_ShowString(30,40,"+"),
								OLED_ShowNumber(36,40,MotorB.Motor_Pwm,4,12);
										
								OLED_ShowString(66,40,"R-E:");

		if(MotorB.Target_Encoder<0)		OLED_ShowString(98,40,"-");
		if(MotorB.Target_Encoder>=0)  	OLED_ShowString(98,40,"+");
									OLED_ShowNumber(104,40,MotorB.Target_Encoder*1000,4,12);
		//=============��������ʾģʽ����ѹ��������=======================//
		
//		if(Car_Num == Akm_Car)				OLED_ShowString(0,50,"Akm  ");
//		else if(Car_Num == Diff_Car)		OLED_ShowString(0,50,"Diff ");
//		else if(Car_Num == Small_Tank_Car)	OLED_ShowString(0,50,"STank");
//		else 								OLED_ShowString(0,50,"BTank");
								OLED_ShowString(0,50,"CCD ");				//ģʽ��ʾ
								OLED_ShowString(54,50,".");
								OLED_ShowString(78,50,"V");
								OLED_ShowNumber(40,50,Voltage/100,2,12);
								OLED_ShowNumber(62,50,Voltage/10%10,1,12);//��ʾ��ѹ
								OLED_ShowNumber(70,50,Voltage%10,1,12);
		if(Flag_Stop)          	OLED_ShowString(105,50, "OFF");
		if(!Flag_Stop)        	OLED_ShowString(105,50, "ON ");

	}
	else
	{
		//==================�ڶ�����ʾС��ģʽ��ʽ==================//	
												OLED_ShowString(0,0, "Mode:");	//��ʾģʽ
		if(Mode==Normal_Mode)					OLED_ShowString(50,0,"Normal ");
		else if(Mode==Lidar_Avoid_Mode)			OLED_ShowString(50,0,"Avoid  ");
		else if(Mode==Lidar_Follow_Mode)		OLED_ShowString(50,0,"Follow ");
		else if(Mode==Lidar_Along_Mode)		  OLED_ShowString(50,0,"Along  ");
//		else if(Mode==Measure_Distance_Mode)	OLED_ShowString(50,0,"Measure");//��������������

		//====================�ڶ�����ʾС�����Ʒ�ʽ================//
			if(APP_ON_Flag==RC_ON)		OLED_ShowString(0,10,"Bluetooth ");	//����
			else if(PS2_ON_Flag==RC_ON)	OLED_ShowString(0,10,"PS2       ");	//PS2�ֱ�
		  else if(ROS_ON_Flag==RC_ON) OLED_ShowString(0,10,"ROS       ");	//ROS
			else						OLED_ShowString(0,10,"LINKING...");	//������û�����ӣ���ʾ��������
//			if(Mode==Measure_Distance_Mode)
//										OLED_ShowString(110,10,"mm"),		
//										OLED_ShowNumber(84,10,Distance1,4,12);//��ʾ���������Եľ���
									  
		//================��������ʾ�ٶ�====================//	
		if(Car_Num == Akm_Car)//����������ת���ٶ���ʾ�Ƕ�
		{
									OLED_ShowString(00,20,"SPEED:");
			if( RC_Velocity<0)		OLED_ShowString(50,20,"-");
			else					OLED_ShowString(50,20,"+");
									OLED_ShowNumber(60,20,myabs(RC_Velocity),4,12);
			
			if(APP_ON_Flag == RC_ON)
				OLED_ShowString(98,20,"Pi/5");
			else if(PS2_ON_Flag == RC_ON)
				OLED_ShowString(98,20,"Pi/4");
		}
		else//��������ת���ٶ���ʾ����ת���ٶ�
		{
									OLED_ShowString(00,20,"SPEED:");
			if( RC_Velocity<0)		OLED_ShowString(50,20,"-");
			else					OLED_ShowString(50,20,"+");
									OLED_ShowNumber(60,20,myabs(RC_Velocity),4,12);
			if( RC_Turn_Velocity<0)		OLED_ShowString(94,20,"-");
			else					OLED_ShowString(94,20,"+");
									OLED_ShowNumber(106,20,myabs(RC_Turn_Velocity),3,12);
			
		}

		//=============��������ʾ����PWM���ٶ�����=======================//		
								OLED_ShowString(00,30,"L-P:");
		if(MotorA.Current_Encoder<0)	{OLED_ShowString(30,30,"-"),
								OLED_ShowNumber(36,30,-MotorA.Current_Encoder*1000,4,12);}
		if(MotorA.Current_Encoder>=0)	{OLED_ShowString(30,30,"+"),
								OLED_ShowNumber(36,30,MotorA.Current_Encoder*1000,4,12);}
										
								OLED_ShowString(66,30,"L-E:");

		if(MotorA.Target_Encoder<0)	  	{OLED_ShowString(94,30,"-");OLED_ShowNumber(104,30,-MotorA.Target_Encoder*1000,4,12);}
		if(MotorA.Target_Encoder>=0)  	{OLED_ShowString(94,30,"+");
									OLED_ShowNumber(104,30,MotorA.Target_Encoder*1000,4,12);}

		//=============��������ʾ�ҵ��PWM���ٶ�����=======================//		
								OLED_ShowString(00,40,"R-P:");
		if(MotorB.Current_Encoder<0){	OLED_ShowString(30,40,"-"),
								OLED_ShowNumber(36,40,-MotorB.Current_Encoder*1000,4,12);}
		if(MotorB.Current_Encoder>=0)	{OLED_ShowString(30,40,"+"),
								OLED_ShowNumber(36,40,MotorB.Current_Encoder*1000,4,12);}
										
								OLED_ShowString(66,40,"R-E:");
		if(MotorB.Target_Encoder<0)		{OLED_ShowString(94,40,"-");OLED_ShowNumber(104,40,-MotorB.Target_Encoder*1000,4,12);}
		if(MotorB.Target_Encoder>=0)  	{OLED_ShowString(94,40,"+");
									OLED_ShowNumber(104,40,MotorB.Target_Encoder*1000,4,12);}

		//=============��������ʾ���ͣ���ѹ��������=======================//
		if(Car_Num == Akm_Car)				OLED_ShowString(0,50,"AKM  ");
		else if(Car_Num == Diff_Car)		OLED_ShowString(0,50,"DIFF ");
		else if(Car_Num == Small_Tank_Car)	OLED_ShowString(0,50,"STANK");
		else 								OLED_ShowString(0,50,"BTANK");
	
								OLED_ShowString(62,50,".");
								OLED_ShowString(86,50,"V");
								OLED_ShowNumber(48,50,Voltage/100,2,12);
								OLED_ShowNumber(70,50,Voltage/10%10,1,12);//��ʾ��ѹ
								OLED_ShowNumber(78,50,Voltage%10,1,12);
		if(Flag_Stop)          	OLED_ShowString(105,50, "OFF");
		else        	          OLED_ShowString(105,50, "ON ");
										
	}
	//==================ˢ��==================//
	OLED_Refresh_Gram();	
}

/**************************************************************************
Function: OLED_Show_CCD
Input   : none
Output  : none
�������ܣ�CCDģʽ��ʾ����������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	

void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	u8 i=0;
	OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	for(i = 0;i<8; i++)
	{
		OLED_DrawPoint(x,y+i,t);
	}
}

void OLED_Show_CCD(void)
{ 
	u8 i,t;
	for(i = 0;i<128; i++)
	{
		if(CCD_ADV[i]<CCD_Threshold) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
	}
}

/**************************************************************************
Function: Car_Select_Show
Input   : none
Output  : none
�������ܣ�����ѡ����ʾ
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void Car_Select_Show(void)
{						  
	OLED_ShowString(0,00,"Rotate Resistor");
	OLED_ShowString(0,10,"To Select Car");
	OLED_ShowString(0,20,"Current Car Is");
	if(Car_Num==Diff_Car)         			OLED_ShowString(24,30," Diff Car ");//����С��
	if(Car_Num==Akm_Car)        			OLED_ShowString(24,30,"  Akm Car ");//������
	if(Car_Num==Small_Tank_Car)				OLED_ShowString(24,30,"S_Tank Car");//С�Ĵ���
	if(Car_Num==Big_Tank_Car)				OLED_ShowString(24,30,"B_Tank Car");//���Ĵ���
	
	OLED_ShowString(0,40,"Press User Key");
	OLED_ShowString(0,50,"To End Selection");
	
	OLED_Refresh_Gram();	
}

/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	static u8 flag;
	int Velocity_Left_Show,Velocity_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1000)*2/3;			if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //�Ե�ѹ���ݽ��д���
	Velocity_Right_Show=(int)(OriginalEncoder.B*1.1); 	if(Velocity_Right_Show<0) Velocity_Right_Show=-Velocity_Right_Show;			  //�Ա��������ݾ������ݴ������ͼ�λ�
	Velocity_Left_Show= (int)(OriginalEncoder.A*1.1);  	if(Velocity_Left_Show<0) Velocity_Left_Show=-Velocity_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//����PID����,��APP���ν�����ʾ
	{
		 printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)RC_Velocity,(int)CCD_Move_X,(int)0,(int)0,(int)0,(int)0,(int)0,0,0);//��ӡ��APP����
		 PID_Send=0;		
  }
   else	if(flag==0)		// ���͵�ص�ѹ���ٶȣ��ǶȵȲ�������APP��ҳ��ʾ
	   printf("{A%d:%d:%d:%d}$",(int)Velocity_Left_Show,(int)Velocity_Right_Show,(int)Voltage_Show,(int)0); //��ӡ��APP����
	 else				//����С����̬�ǣ��ڲ��ν�����ʾ
	    printf("{B%d:%d}$",(int)MotorA.Current_Encoder,(int)MotorB.Current_Encoder); //��ʾ���ұ��������ٶ�																	   //�ɰ���ʽ����������ʾ���Σ�������ʾ���
}


//��λ��ʾ����
/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DataScope(void)
{   
	u8 i;			//��������
	u8 Send_Count;	//������Ҫ���͵����ݸ���
	float Vol;		//��ѹ����
	Vol=(float)Voltage/100;
	DataScope_Get_Channel_Data( MotorA.Velocity, 1 );     	//��ʾ�����ٶȣ���λmm/s
	DataScope_Get_Channel_Data( MotorB.Velocity, 2 );    	//��ʾ�����ٶ�
	DataScope_Get_Channel_Data( Vol, 3 );               	//��ʾ��ص�ѹ ��λ��V
//	DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
//	DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
//	DataScope_Get_Channel_Data(0, 7 );
//	DataScope_Get_Channel_Data( 0, 8 ); 
//	DataScope_Get_Channel_Data(0, 9 );  
//	DataScope_Get_Channel_Data( 0 , 10);
	Send_Count = DataScope_Data_Generate(CHANNEL_NUMBER);//CHANNEL_NUMBER�ɸı�ͨ��������Ŀǰ��4
	for( i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��uint
��    �ߣ�ƽ��С��֮��
**************************************************************************/
u16 myabs(int Input)
{
	int Output;
	if(Input > 0)
		Output = Input;
	else
		Output = -Input;
	return Output;
}



/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	u8 Car_Select_Count = 0;
	u32 Car_Select_Sum = 0;				//����ѡ��ADC��ر���
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADCֵ�ֶα�����ȡ����С���ͺ�����
	//����ѡ���λ�ƣ�adcÿ10��ȡһ��ƽ��
		for(Car_Select_Count=0;Car_Select_Count<2;Car_Select_Count++)
		{
			Car_Select_Sum += Get_Adc(CAR_ADC_CHANNEL);
			//delay_ms(1);
		}
		Car_Num = Car_Select_Sum/2/(Max_Car_ADC/Num_Of_Car);	//������͵ĺ���
		Car_Select_Sum = 0;
	
}

