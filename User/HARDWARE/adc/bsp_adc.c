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

#include "./adc/bsp_adc.h"

__IO uint16_t ADC_ConvertedValue;
/**************************************************************************
Function: ADCx_GPIO_Config
Input   : none
Output  : none
�������ܣ�ADC�˿ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//ADC�˿ڳ�ʼ�������ڶ�ȡ��ѹ
static void ADCx_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// �� ADC IO�˿�ʱ��
	ADC_GPIO_APBxClock_FUN ( ADC_GPIO_CLK, ENABLE );
	
	// ���� ADC IO ����ģʽ
	// ����Ϊģ������
	GPIO_InitStructure.GPIO_Pin = ADC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	
	// ��ʼ�� ADC IO
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);				
}

/**************************************************************************
Function: ADCx_Mode_Config
Input   : none
Output  : none
�������ܣ�ADCģʽ��ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//ADC��ʼ�������ڶ�ȡ��ѹ��
//�жϷ�ʽ
static void ADCx_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	

	// ��ADCʱ��
	ADC_APBxClock_FUN ( ADC_CLK, ENABLE );
	
	//��λADC2,������ ADC2 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_DeInit(ADCx); 

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
	ADC_Init(ADCx, &ADC_InitStructure);
	
	// ����ADCʱ��ΪPCLK2��8��Ƶ����9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	// ���� ADC ͨ��ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL, 1, 
	                         ADC_SampleTime_239Cycles5);
	
	// ADC ת�����������жϣ����жϷ�������ж�ȡת��ֵ
	ADC_ITConfig(ADCx, ADC_IT_EOC, ENABLE);
	
	// ����ADC ������ʼת��
	ADC_Cmd(ADCx, ENABLE);
	
	// ��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ADCx);
	// �ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ADCx));
	
	// ADC��ʼУ׼
	ADC_StartCalibration(ADCx);
	// �ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADCx));
	
	// ����û�в����ⲿ����������ʹ���������ADCת�� 
	ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}
/**************************************************************************
Function: ADC_NVIC_Config
Input   : none
Output  : none
�������ܣ�ADC�ж�����
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//ADC�ж����ã����ڶ�ȡ��ѹ
static void ADC_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	// ���ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	// �����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/**************************************************************************
Function: Voltage_ADC_Init
Input   : none
Output  : none
�������ܣ�ADC��ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//���ڶ�ȡ��ѹ
void Voltage_ADC_Init(void)
{
	ADCx_GPIO_Config();		//�˿�����
	ADCx_Mode_Config();		//ģʽ����
	ADC_NVIC_Config();		//�ж�����
}

/**************************************************************************
Function: Car_Select_ADC_GPIO_Config
Input   : none
Output  : none
�������ܣ�ADC�˿ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//����ѡ��ADC�˿ڳ�ʼ����������ѡ����
static void Car_Select_ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// �� ADC IO�˿�ʱ��
	RCC_APB2PeriphClockCmd ( CAR_ADC_GPIO_CLK, ENABLE );
	
	// ���� ADC IO ����ģʽ
	// ����Ϊģ������
	GPIO_InitStructure.GPIO_Pin = CAR_ADC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	
	// ��ʼ�� ADC IO
	GPIO_Init(CAR_ADC_PORT, &GPIO_InitStructure);				
}
/**************************************************************************
Function: Car_Select_ADC_Mode_Config
Input   : none
Output  : none
�������ܣ�ADCģʽ��ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//����ѡ��ADC�˿ڳ�ʼ����������ѡ����
static void Car_Select_ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	

	// ��ADCʱ��
	CAR_ADC_APBxClock_FUN ( CAR_ADC_CLK, ENABLE );
	
	 //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_DeInit(CAR_ADC); 

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
	ADC_Init(CAR_ADC, &ADC_InitStructure);
	
	// ����ADCʱ��ΪPCLK2��6��Ƶ����12MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	
	// ���� ADC ͨ��ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(CAR_ADC, CAR_ADC_CHANNEL, 1, 
	                         ADC_SampleTime_239Cycles5);
							 
	// ���ж�
	ADC_ITConfig(CAR_ADC, ADC_IT_EOC, DISABLE);
	
	// ����ADC ������ʼת��
	ADC_Cmd(CAR_ADC, ENABLE);
	
	// ��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(CAR_ADC);
	// �ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(CAR_ADC));
	
	// ADC��ʼУ׼
	ADC_StartCalibration(CAR_ADC);
	// �ȴ�У׼���
	while(ADC_GetCalibrationStatus(CAR_ADC));
	
}

/**************************************************************************
Function: Car_Select_ADC_Init
Input   : none
Output  : none
�������ܣ�ADCģʽ��ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//����ѡ��ADC��ʼ����������ѡ����
void Car_Select_ADC_Init(void)
{
	Car_Select_ADC_GPIO_Config();	//�˿�����
	Car_Select_ADC_Mode_Config();	//ģʽ����
}


/**************************************************************************
Function: ADC_IRQHandler
Input   : none
Output  : none
�������ܣ���ѹ��ȡADC�жϺ���
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
void ADC_IRQHandler(void)
{	
	if (ADC_GetITStatus(ADCx,ADC_IT_EOC)==SET) 
	{
		// ��ȡADC��ת��ֵ
		ADC_ConvertedValue = ADC_GetConversionValue(ADCx);
		ADC_ClearITPendingBit(ADCx,ADC_IT_EOC);
	}
}



/**************************************************************************
Function: Get_Voltage
Input   : none
Output  : none
�������ܣ���ȡ��ѹ
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
u16 Get_Voltage(void)
{
	u16 Voltage;

	ADC_SoftwareStartConvCmd(ADCx, ENABLE);//����ת������
	
	Voltage = Max_Voltage/Max_Voltage_ADC*ADC_ConvertedValue*Ratio*100;//��ȡ��ѹ���Ŵ�100������
	
	return Voltage;
}
/**************************************************************************
Function: Get_Voltage
Input   : none
Output  : none
�������ܣ���ȡADC��ֵ
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//��ȡADC��ֵ����Ҫ���ڳ��͵�ѡ��
u16 Get_Adc(u8 ch)   
{
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	__nop();
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}


/********************************END OF FILE***********************************/
