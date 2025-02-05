#include "ELE_CCD.h"

int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
u16  CCD_ADV[128]={0};
u8 CCD_Median,CCD_Threshold;                 //����CCD���

//���Ѳ��ADC�˿ڳ�ʼ��
/**************************************************************************
Function: ELE_ADC_GPIO_Config
Input   : none
Output  : none
�������ܣ���ʼ�����Ѳ��GPIO
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
static void ELE_ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// �� ADC IO�˿�ʱ��
	RCC_APB2PeriphClockCmd ( ELE_ADC_L_GPIO_CLK|ELE_ADC_M_GPIO_CLK|ELE_ADC_R_GPIO_CLK, ENABLE );
	
	// ���� ADC IO ����ģʽ
	// ����Ϊģ������
	//��·�˿����ã�PA4
	GPIO_InitStructure.GPIO_Pin = ELE_ADC_L_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ELE_ADC_L_PORT, &GPIO_InitStructure);	

	//�м�˿����ã�PA5
	GPIO_InitStructure.GPIO_Pin = ELE_ADC_M_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ELE_ADC_M_PORT, &GPIO_InitStructure);				


	//��·�˿����ã�PC5
	GPIO_InitStructure.GPIO_Pin = ELE_ADC_R_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ELE_ADC_R_PORT, &GPIO_InitStructure);				
	
}

/**************************************************************************
Function: ELE_ADC_Mode_Config
Input   : none
Output  : none
�������ܣ���ʼ�����Ѳ��ADC
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	

static void ELE_ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	

	// ��ADCʱ��
	ELE_ADC_APBxClock_FUN ( ELE_ADC_CLK, ENABLE );
	
	//��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_DeInit(ELE_ADC); 

	// ADC ģʽ����
	// ֻʹ��һ��ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	// ��ֹɨ��ģʽ����ͨ����Ҫ����ͨ������Ҫ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

	// ����ת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

	// �����ⲿ����ת���������������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

	// ת������Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// ת��ͨ��1��
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
		
	// ��ʼ��ADC
	ADC_Init(ELE_ADC, &ADC_InitStructure);
	
	// ����ADCʱ��ΪPCLK2��6��Ƶ����12MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	// ���� ADC ͨ��ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ELE_ADC, ELE_ADC_L_CHANNEL, 1, 
	                         ADC_SampleTime_239Cycles5);
							 
	ADC_RegularChannelConfig(ELE_ADC, ELE_ADC_M_CHANNEL, 1, 
	                         ADC_SampleTime_239Cycles5);

	ADC_RegularChannelConfig(ELE_ADC, ELE_ADC_R_CHANNEL, 1, 
	                         ADC_SampleTime_239Cycles5);
		
	// ���ж�
	ADC_ITConfig(ELE_ADC, ADC_IT_EOC, DISABLE);
	
	// ����ADC ������ʼת��
	ADC_Cmd(ELE_ADC, ENABLE);
	
	// ��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ELE_ADC);
	// �ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ELE_ADC));
	
	// ADC��ʼУ׼
	ADC_StartCalibration(ELE_ADC);
	// �ȴ�У׼���
	while(ADC_GetCalibrationStatus(ELE_ADC));
	
}

/**************************************************************************
Function: ELE_ADC_Init
Input   : none
Output  : none
�������ܣ���ʼ�����Ѳ��ADC
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
//���Ѳ�߳�ʼ��
void ELE_ADC_Init(void)
{
	ELE_ADC_GPIO_Config();
	ELE_ADC_Mode_Config();

}


/**************************************************************************
Function: CCD_ADC_Mode_Config
Input   : none
Output  : none
�������ܣ���ʼ��CCDѲ��ADC
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
static void CCD_ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	

	// ��ADCʱ��
	CCD_ADC_APBxClock_FUN ( CCD_ADC_CLK, ENABLE );
	
	//��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_DeInit(CCD_ADC); 

	// ADC ģʽ����
	// ֻʹ��һ��ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	// ��ֹɨ��ģʽ����ͨ����Ҫ����ͨ������Ҫ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

	// ����ת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

	// �����ⲿ����ת���������������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

	// ת������Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// ת��ͨ��1��
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
		
	// ��ʼ��ADC
	ADC_Init(CCD_ADC, &ADC_InitStructure);
	
	// ����ADCʱ��ΪPCLK2��6��Ƶ����12MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

	// ���� ADC ͨ��ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(CCD_ADC, CCD_ADC_CHANNEL, 1, 
	                         ADC_SampleTime_239Cycles5);
		
	// ���ж�
	ADC_ITConfig(CCD_ADC, ADC_IT_EOC, DISABLE);
	
	// ����ADC ������ʼת��
	ADC_Cmd(CCD_ADC, ENABLE);
	
	// ��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(CCD_ADC);
	// �ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(CCD_ADC));
	
	// ADC��ʼУ׼
	ADC_StartCalibration(CCD_ADC);
	// �ȴ�У׼���

	while(ADC_GetCalibrationStatus(CCD_ADC));
}


/**************************************************************************
Function: CCD_GPIO_Config
Input   : none
Output  : none
�������ܣ���ʼ��CCDѲ��GPIO
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
static void CCD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// ��CCD�˿�ʱ��
	RCC_APB2PeriphClockCmd ( TSL_CLK_GPIO_CLK|TSL_SI_GPIO_CLK|CCD_ADC_GPIO_CLK, ENABLE );
	
	// CLK,SI����Ϊ���	
	GPIO_InitStructure.GPIO_Pin = TSL_SI_PIN;				//PA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(TSL_SI_PORT, &GPIO_InitStructure);	


	GPIO_InitStructure.GPIO_Pin = TSL_CLK_PIN;				//PA5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(TSL_CLK_PORT, &GPIO_InitStructure);				


	//����ADC����ģʽ
	GPIO_InitStructure.GPIO_Pin = CCD_ADC_PIN;				//PC5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(CCD_ADC_PORT, &GPIO_InitStructure);				
	
}


/**************************************************************************
Function: CCD_Init
Input   : none
Output  : none
�������ܣ���ʼ��CCDѲ��
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void CCD_Init(void)
{
	CCD_GPIO_Config();
	CCD_ADC_Mode_Config();

}

/**************************************************************************
Function: ELE_Mode
Input   : none
Output  : none
�������ܣ����Ѳ��ģʽ����
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
//void ELE_Mode(void)
//{
//	int Sum = 0;
//	Sensor_Left = Get_Adc(ELE_ADC_L_CHANNEL);
//	Sensor_Middle = Get_Adc(ELE_ADC_M_CHANNEL);
//	Sensor_Right = Get_Adc(ELE_ADC_R_CHANNEL);
//	Sum = Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;			
//	Sensor = Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);
//	if(Detect_Barrier() == No_Barrier)		//��⵽���ϰ���
//	{
//		Move_X = ELE_Move_X;				//Ѳ�ߵ�ʱ���ٶ�,Ĭ����0.25m
//		if(Car_Num == Diff_Car|| Car_Num == Akm_Car)
//			Move_Z = ELE_PID(Sensor,92);		//Ŀ��ֵ92
//		else
//			Move_Z = ELE_PID(Sensor,100);		//Ŀ��ֵ100
//		Buzzer_Alarm(0);
//    }											
//	else									//���ϰ���
//	{
//		if(!Flag_Stop)
//			Buzzer_Alarm(100);				//�����ʹ�ܵ�ʱ�����ϰ��������������
//		else 
//			Buzzer_Alarm(0);
//		Move_X = 0;
//		Move_Z = 0;
//	}
//}

/**************************************************************************
Function: Detect_Barrier
Input   : none
Output  : 1or0(Barrier_Detected or No_Barrier)
�������ܣ����Ѳ��ģʽ�״����ϰ���
��ڲ���: ��
����  ֵ��1��0(��⵽�ϰ�������ϰ���)
**************************************************************************/	 	
//����ϰ���
u8 Detect_Barrier(void)
{
	int i;
	u8 point_count = 0;
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<390;i++)	//����Ƿ����ϰ���
		{
			if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<700)//700mm���Ƿ����ϰ���
					point_count++;
		  }
		}
		if(point_count > 3)//���ϰ���
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}
/**************************************************************************
�������ܣ������ʱ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}

/**************************************************************************
Function: Read_TSL
Input   : none
Output  : none
�������ܣ���ȡCCDģ�������
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
//��ȡCCDģ�������
void RD_TSL(void) 
{
	u8 i=0,tslp=0;
	
	TSL_CLK_LOW;
	TSL_SI_LOW;
	Dly_us();

	TSL_SI_HIGH;
	Dly_us();

	TSL_CLK_LOW;
	Dly_us();
	
	TSL_CLK_HIGH;
	Dly_us();
	
	TSL_SI_LOW;
	Dly_us();
	
	for(i=0;i<128;i++)
	{ 
		TSL_CLK_LOW; 
		Dly_us();  //�����ع�ʱ��
		CCD_ADV[tslp]=(Get_Adc(CCD_ADC_CHANNEL))>>4;		
		++tslp;
		TSL_CLK_HIGH;
		Dly_us();  
	}
}


/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Median(void)
{ 
	static u8 i,j,Left,Right,Last_CCD_Median;
	static u16 value1_max,value1_min;

	value1_max=CCD_ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
	for(i=5;i<123;i++)   //���߸�ȥ��5����
	{
		if(value1_max<=CCD_ADV[i])
		value1_max=CCD_ADV[i];
	}
	value1_min=CCD_ADV[0];  //��Сֵ
	for(i=5;i<123;i++) 
	{
		if(value1_min>=CCD_ADV[i])
		value1_min=CCD_ADV[i];
	}
	CCD_Threshold=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
	
//	//CCDѲ����
//	for(i = 15;i<118; i++)   //Ѱ�����������
//	{
//		if(CCD_ADV[i]>CCD_Threshold&&CCD_ADV[i+1]>CCD_Threshold&&CCD_ADV[i+2]>CCD_Threshold&&CCD_ADV[i+3]<CCD_Threshold&&CCD_ADV[i+4]<CCD_Threshold&&CCD_ADV[i+5]<CCD_Threshold)
//		{	
//			Left=i;
//			break;	
//		}
//	}
//	for(j = 118;j>15; j--)//Ѱ���ұ�������
//	{
//		if(CCD_ADV[j]<CCD_Threshold&&CCD_ADV[j+1]<CCD_Threshold&&CCD_ADV[j+2]<CCD_Threshold&&CCD_ADV[j+3]>CCD_Threshold&&CCD_ADV[j+4]>CCD_Threshold&&CCD_ADV[j+5]>CCD_Threshold)
//		{	
//			Right=j;
//			break;	
//		}
//	}
	 //CCDѲ˫�ߣ������ҵ���ߺ��ߵ��ұ�ֵ���ҵ��ұߺ��ߵ����ֵ��Ȼ��ȷ����ֵ
    for(i=59;i>5;i--)
	{
		if(CCD_ADV[i]<CCD_Threshold&&CCD_ADV[i+1]<CCD_Threshold&&CCD_ADV[i+2]<CCD_Threshold&&CCD_ADV[i+3]>CCD_Threshold&&CCD_ADV[i+4]>CCD_Threshold&&CCD_ADV[i+5]>CCD_Threshold)
		{
			Left=i;
			break;
		}
	}
	for(j=60;j<118;j++)
  {
	  if(CCD_ADV[j]<CCD_Threshold&&CCD_ADV[j+1]<CCD_Threshold&&CCD_ADV[j+2]<CCD_Threshold&&CCD_ADV[j+3]>CCD_Threshold&&CCD_ADV[j+4]>CCD_Threshold&&CCD_ADV[j+5]>CCD_Threshold)
	  {
		  Right=j;
		  break;
	  }
  }
	CCD_Median=(Right+Left)/2;//��������λ��
	if(myabs(CCD_Median-Last_CCD_Median)>90)   //�������ߵ�ƫ����̫��
		CCD_Median=Last_CCD_Median;    //��ȡ��һ�ε�ֵ
	Last_CCD_Median=CCD_Median;  //������һ�ε�ƫ��

}	
/**************************************************************************
Function: CCD_Mode
Input   : none
Output  : none
�������ܣ�CCDѲ��ģʽ����
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
//void CCD_Mode(void)
//{
//	RD_TSL();			//��ȡCCDģ������
//	Find_CCD_Median();	//����ֵ
//	
//	Move_X = CCD_Move_X;			//CCDѲ���ٶȣ�Ĭ��0.3m
//	Move_Z = CCD_PID(CCD_Median,64);//PID���ڣ�Ŀ��ֵ64
//	
//}


