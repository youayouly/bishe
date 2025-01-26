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

//���ģ��Ĭ�ϲ�ʹ��
//4·������ģ��
//�� 4·��ģң��
#include "capture.h"

u16 Distance1,Distance2,Distance3,Distance4;	//4�����������������

// ��ʱ�����벶���û��Զ�������ṹ�嶨��
TIM_ICUserValueTypeDef Distance_TIM2_CH2_ICUserValueStructure = {0,0,0,0};//������ģ��1������Ӧ�ı���
TIM_ICUserValueTypeDef Distance_TIM2_CH3_ICUserValueStructure = {0,0,0,0};//������ģ��2������Ӧ�ı���
TIM_ICUserValueTypeDef Distance_TIM2_CH4_ICUserValueStructure = {0,0,0,0};//������ģ��3������Ӧ�ı���
TIM_ICUserValueTypeDef Distance_TIM1_CH4_ICUserValueStructure = {0,0,0,0};//������ģ��4������Ӧ�ı���


TIM_ICUserValueTypeDef PWM_TIM2_CH4_ICUserValueStructure = {0,0,0,0};//��ģң�ص�һ·
TIM_ICUserValueTypeDef PWM_TIM2_CH3_ICUserValueStructure = {0,0,0,0};//��ģң�صڶ�·
TIM_ICUserValueTypeDef PWM_TIM1_CH4_ICUserValueStructure = {0,0,0,0};//��ģң�ص���·
TIM_ICUserValueTypeDef PWM_TIM1_CH1_ICUserValueStructure = {0,0,0,0};//��ģң�ص���·


/**************************************************************************
Function: Distance_Capture_GPIO_Config
Input   : none
Output  : none
�������ܣ��������˿ڳ�ʼ��
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void Distance_Capture_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(CAPTURE_TIM2_CH2_GPIO_CLK|CAPTURE_TRIG_GPIO_CLK1,ENABLE);//������ģ��1�˿�ʹ��
	RCC_APB2PeriphClockCmd(CAPTURE_TIM2_CH3_GPIO_CLK|CAPTURE_TRIG_GPIO_CLK2,ENABLE);//������ģ��2�˿�ʹ��
	RCC_APB2PeriphClockCmd(CAPTURE_TIM2_CH4_GPIO_CLK|CAPTURE_TRIG_GPIO_CLK3,ENABLE);//������ģ��3�˿�ʹ��
	RCC_APB2PeriphClockCmd(CAPTURE_TIM1_CH4_GPIO_CLK|CAPTURE_TRIG_GPIO_CLK4,ENABLE);//������ģ��4�˿�ʹ��

	//������ģ��1
	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TIM2_CH2_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA1 ����  
	GPIO_Init(CAPTURE_TIM2_CH2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TRIG_PIN1;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     		//PC15��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;    
	GPIO_Init(CAPTURE_TRIG_PORT1, &GPIO_InitStructure);

	//������ģ��2
	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TIM2_CH3_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA2 ����  
	GPIO_Init(CAPTURE_TIM2_CH3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TRIG_PIN2;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     		//PA12��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;    
	GPIO_Init(CAPTURE_TRIG_PORT2, &GPIO_InitStructure);
	
	//������ģ��3
	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TIM2_CH4_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA3 ����  
	GPIO_Init(CAPTURE_TIM2_CH4_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TRIG_PIN3;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     		//PB13��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;    
	GPIO_Init(CAPTURE_TRIG_PORT3, &GPIO_InitStructure);
	
	//������ģ��4
	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TIM1_CH4_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA11 ����  
	GPIO_Init(CAPTURE_TIM1_CH4_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = CAPTURE_TRIG_PIN4;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     		//PB12��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;    
	GPIO_Init(CAPTURE_TRIG_PORT4, &GPIO_InitStructure);
}


/**************************************************************************
Function: Distance_Capture_Mode_Config
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ��������˿ڳ�ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void Distance_Capture_Mode_Config(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	//ʹ�ܶ�ʱ��ʱ�ӡ�TIM2
	CAPTURE_TIM2_APBxClock_FUN(CAPTURE_TIM2_CLK,ENABLE);				
	//ʹ�ܶ�ʱ��ʱ�ӡ�TIM1
	CAPTURE_TIM1_APBxClock_FUN(CAPTURE_TIM1_CLK,ENABLE);
	
	/*ʱ����ʱ����ʼ��*/
	//��ʱ��2
	TIM_TimeBaseStructure.TIM_Period = arr; 					//�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(CAPTURE_TIM2, &TIM_TimeBaseStructure); 	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʱ��1
	TIM_TimeBaseStructure.TIM_Period = arr; 					//�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(CAPTURE_TIM1, &TIM_TimeBaseStructure); 	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	/*����ͨ����ʼ��*/
	//��ʱ��2��CH2��CH3��CH4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 			//ͨ��2
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(CAPTURE_TIM2, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 			//ͨ��3
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(CAPTURE_TIM2, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 			//ͨ��4
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(CAPTURE_TIM2, &TIM_ICInitStructure);

	//��ʱ��1��CH4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 			//ͨ��4
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(CAPTURE_TIM1, &TIM_ICInitStructure);

	//ʹ���ж�����
	//TIM2_CH2,CH3,CH4
	TIM_ITConfig(CAPTURE_TIM2,TIM_IT_Update|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);	//��������жϺͲ����ж�	
	//TIM1_CH4
	TIM_ITConfig(CAPTURE_TIM1,TIM_IT_Update|TIM_IT_CC4,ENABLE);	//��������ж� ,����CC4IE�����ж�	

	//��ʱ��ʹ��
	TIM_Cmd(CAPTURE_TIM2,ENABLE); 										//ʹ�ܶ�ʱ��2
	
	TIM_Cmd(CAPTURE_TIM1,ENABLE); 										//ʹ�ܶ�ʱ��1
	
}

/**************************************************************************
Function: Distance_Capture_NVIC_Config
Input   : none
Output  : none
�������ܣ��������жϳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	

void Distance_Capture_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//�ж����ȼ�����
	//TIM2���ж�
	NVIC_InitStructure.NVIC_IRQChannel = CAPTURE_TIM2_IRQ;  	//TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  		//�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

	//TIM1�����жϺͲ����ж�
	NVIC_InitStructure.NVIC_IRQChannel = CAPTURE_TIM1_CC_IRQn;  //TIM1�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  		//�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

	NVIC_InitStructure.NVIC_IRQChannel = PWM_TIM1_UP_IRQn;  	//TIM1�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  		//�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

}




/**************************************************************************
Function: Distance_Cap_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ���������ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void Distance_Cap_Init(u16 arr,u16 psc)
{
	Distance_Capture_GPIO_Config();
	Distance_Capture_NVIC_Config();
	Distance_Capture_Mode_Config(arr,psc);
}





/**************************************************************************
Function: PWM_Capture_GPIO_Config
Input   : none
Output  : none
�������ܣ���ģң�ض˿ڳ�ʼ��
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void PWM_Capture_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(PWM_TIM2_CH4_GPIO_CLK,ENABLE);//��ģң�ص�һ·�ӿ�
	RCC_APB2PeriphClockCmd(PWM_TIM2_CH3_GPIO_CLK,ENABLE);//��ģң�صڶ�·�ӿ�
	RCC_APB2PeriphClockCmd(PWM_TIM1_CH4_GPIO_CLK,ENABLE);//��ģң�ص���·�ӿ�
	RCC_APB2PeriphClockCmd(PWM_TIM1_CH1_GPIO_CLK,ENABLE);//��ģң�ص���·�ӿ�

	//��һ·
	GPIO_InitStructure.GPIO_Pin  = PWM_TIM2_CH4_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA1 ����  
	GPIO_Init(PWM_TIM2_CH4_PORT, &GPIO_InitStructure);


	//�ڶ�·
	GPIO_InitStructure.GPIO_Pin  = PWM_TIM2_CH3_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA2 ����  
	GPIO_Init(PWM_TIM2_CH3_PORT, &GPIO_InitStructure);

	//����·
	GPIO_InitStructure.GPIO_Pin  = PWM_TIM1_CH4_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA3 ����  
	GPIO_Init(PWM_TIM1_CH4_PORT, &GPIO_InitStructure);
	
	//����·
	GPIO_InitStructure.GPIO_Pin  = PWM_TIM1_CH1_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 				//PA11 ����  
	GPIO_Init(PWM_TIM1_CH1_PORT, &GPIO_InitStructure);
}

/**************************************************************************
Function: PWM_Capture_Mode_Config
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ���ģң�ز���PWM�ߵ�ƽ��ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void PWM_Capture_Mode_Config(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	//ʹ�ܶ�ʱ��ʱ�ӡ�TIM2
	PWM_TIM2_APBxClock_FUN(PWM_TIM2_CLK,ENABLE);				
	//ʹ�ܶ�ʱ��ʱ�ӡ�TIM1
	PWM_TIM1_APBxClock_FUN(PWM_TIM1_CLK,ENABLE);
	
	/*ʱ����ʱ����ʼ��*/
	//��ʱ��2
	TIM_TimeBaseStructure.TIM_Period = arr; 					//�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(PWM_TIM2, &TIM_TimeBaseStructure); 		//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʱ��1
	TIM_TimeBaseStructure.TIM_Period = arr; 					//�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(PWM_TIM1, &TIM_TimeBaseStructure); 		//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	/*����ͨ����ʼ��*/
	//��ʱ��2CH3��CH4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 			//ͨ��3
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(PWM_TIM2, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 			//ͨ��4
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(PWM_TIM2, &TIM_ICInitStructure);

	//��ʱ��1CH4��CH3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 			//ͨ��4
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(PWM_TIM1, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 			//ͨ��4
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;					//���������˲��� ���˲�
	TIM_ICInit(PWM_TIM1, &TIM_ICInitStructure);

	//ʹ���ж�����
	//TIM2_CH3,CH4
	TIM_ITConfig(PWM_TIM2,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,ENABLE);	//��������жϺͲ����ж�	
	//TIM1_CH3��CH4
	TIM_ITConfig(PWM_TIM1,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,ENABLE);	//��������ж� ,����CC4IE�����ж�	

	//��ʱ��ʹ��
	TIM_Cmd(PWM_TIM2,ENABLE); 										//ʹ�ܶ�ʱ��2
	
	TIM_Cmd(PWM_TIM1,ENABLE); 										//ʹ�ܶ�ʱ��1
	
}
/**************************************************************************
Function: PWM_Capture_Mode_Config
Input   : none
Output  : none
�������ܣ���ģң�ز���PWM�ߵ�ƽ��ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	

void PWM_Capture_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//�ж����ȼ�����
	//TIM2���ж�
	NVIC_InitStructure.NVIC_IRQChannel = PWM_TIM2_IRQ;  		//TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  		//�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

	//TIM1�����жϺͲ����ж�
	NVIC_InitStructure.NVIC_IRQChannel = PWM_TIM1_CC_IRQn;  	//TIM1�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  		//�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

	NVIC_InitStructure.NVIC_IRQChannel = CAPTURE_TIM1_UP_IRQn;  //TIM1�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  		//�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

}




/**************************************************************************
Function: PWM_Capture_Mode_Config
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ���ģң�ز���PWM�ߵ�ƽ��ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void PWM_Cap_Init(u16 arr,u16 psc)
{
	PWM_Capture_GPIO_Config();
	PWM_Capture_NVIC_Config();
	PWM_Capture_Mode_Config(arr,psc);
}




/**************************************************************************
Function: Read_Distane
Input   : none
Output  : none
�������ܣ���ȡ������������
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
//��5ms��ʱ�ж��е���
void Read_Distane(void)        
{   
	
	static u16 cnt = 0;	 //������������������Ҫ̫Ƶ������Ծ���Ļ�ȡ��һ����Ӱ��
	u32 temp_distance;
	cnt++;
	if(cnt == 20)		 //���ڴ������ٶ�
	{
		TRIG_HIGH1;      //������ģ��1����   
		delay_us(15);  
		TRIG_LOW1;	
		
		TRIG_HIGH2;      //������ģ��2����   
		delay_us(15);  
		TRIG_LOW2;	

	}
	if(cnt == 40)
	{
		cnt =0;
		TRIG_HIGH3;      //������ģ��3����   
		delay_us(15);  
		TRIG_LOW3;	
		
		TRIG_HIGH4;      //������ģ��4����   
		delay_us(15);  
		TRIG_LOW4;	
	}

	//������ģ��1
	if(Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag)//�ɹ�������һ�θߵ�ƽ
	{
		temp_distance=Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue+Distance_TIM2_CH2_ICUserValueStructure.Capture_Period*65536; //һ���������ٴ�
		Distance1=temp_distance*Light_Speed/2/1000;  						//ʱ��*����/2�����أ� һ������0.001ms
		Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag = 0;			//������һ�β���
	}
	//������ģ��2
	if(Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag)//�ɹ�������һ�θߵ�ƽ
	{
		temp_distance=Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue+Distance_TIM2_CH3_ICUserValueStructure.Capture_Period*65536; //һ���������ٴ�
		Distance2=temp_distance*Light_Speed/2/1000;  						//ʱ��*����/2�����أ� һ������0.001ms
		Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag = 0;			//������һ�β���
	}
	//������ģ��3
	if(Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag)//�ɹ�������һ�θߵ�ƽ
	{
		temp_distance=Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue+Distance_TIM2_CH4_ICUserValueStructure.Capture_Period*65536; //һ���������ٴ�
		Distance3=temp_distance*Light_Speed/2/1000;  						//ʱ��*����/2�����أ� һ������0.001ms
		Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag = 0;			//������һ�β���
	}
	//������ģ��4
	if(Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag)//�ɹ�������һ�θߵ�ƽ
	{
		temp_distance=Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue+Distance_TIM1_CH4_ICUserValueStructure.Capture_Period*65536; //һ���������ٴ�
		Distance4=temp_distance*Light_Speed/2/1000;  						//ʱ��*����/2�����أ� һ������0.001ms
		Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag = 0;			//������һ�β���
	}

}


//ʹ�ó�����
#ifdef Distance_Capture


/**************************************************************************
Function: CAPTURE_TIM2_IRQHandler
Input   : none
Output  : none
�������ܣ��ߵ�ƽ�����жϺ���
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void CAPTURE_TIM2_IRQHandler(void)
{ 	
	// ��Ҫ��������źŵ����ڴ��ڶ�ʱ�������ʱʱ����ʱ���ͻ���������������ж�
	// ���ʱ��������Ҫ�������Ķ�ʱ���ڼӵ������źŵ�ʱ������ȥ
	
	/*************************************ͨ��2*******************************************/
	
	if(Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( CAPTURE_TIM2, TIM_IT_Update) != RESET )               
			Distance_TIM2_CH2_ICUserValueStructure.Capture_Period ++;
		
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (CAPTURE_TIM2, TIM_IT_CC2 ) != RESET)
		{
			// ��һ�β���
			if ( Distance_TIM2_CH2_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture2 (CAPTURE_TIM2);
				// �Զ���װ�ؼĴ������±�־��0
				Distance_TIM2_CH2_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC2PolarityConfig(CAPTURE_TIM2, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				Distance_TIM2_CH2_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture2 (CAPTURE_TIM2)-Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC2PolarityConfig(CAPTURE_TIM2, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				Distance_TIM2_CH2_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (CAPTURE_TIM2,TIM_IT_CC2);	
		}		
	}

	/*************************************ͨ��3*******************************************/
	if(Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( CAPTURE_TIM2, TIM_IT_Update) != RESET )               
			Distance_TIM2_CH3_ICUserValueStructure.Capture_Period ++;
		
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (CAPTURE_TIM2, TIM_IT_CC3 ) != RESET)
		{
			// ��һ�β���
			if ( Distance_TIM2_CH3_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture3 (CAPTURE_TIM2);
				// �Զ���װ�ؼĴ������±�־��0
				Distance_TIM2_CH3_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC3PolarityConfig(CAPTURE_TIM2, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				Distance_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture3 (CAPTURE_TIM2)-Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC3PolarityConfig(CAPTURE_TIM2, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				Distance_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (CAPTURE_TIM2,TIM_IT_CC3);	
		}		
	}
	/*************************************ͨ��4*******************************************/
	if(Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( CAPTURE_TIM2, TIM_IT_Update) != RESET )               
			Distance_TIM2_CH4_ICUserValueStructure.Capture_Period ++;

		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (CAPTURE_TIM2, TIM_IT_CC4 ) != RESET)
		{
			// ��һ�β���
			if ( Distance_TIM2_CH4_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4 (CAPTURE_TIM2);
				// �Զ���װ�ؼĴ������±�־��0
				Distance_TIM2_CH4_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC4PolarityConfig(CAPTURE_TIM2, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				Distance_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4(CAPTURE_TIM2)-Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC4PolarityConfig(CAPTURE_TIM2, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				Distance_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (CAPTURE_TIM2,TIM_IT_CC4);	
		}		
	}
	TIM_ClearITPendingBit (CAPTURE_TIM2,TIM_IT_Update|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);	

}
/**************************************************************************
Function: CAPTURE_TIM1_CC_IRQHandler
Input   : none
Output  : none
�������ܣ��ߵ�ƽ�����жϺ���
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
//������ģ��4�����ж�
void CAPTURE_TIM1_CC_IRQHandler(void)
{
	/*************************************ͨ��4*******************************************/
	if(Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (CAPTURE_TIM1, TIM_IT_CC4 ) != RESET)
		{
			// ��һ�β���
			if ( Distance_TIM1_CH4_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4 (CAPTURE_TIM1);
				// �Զ���װ�ؼĴ������±�־��0
				Distance_TIM1_CH4_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC4PolarityConfig(CAPTURE_TIM1, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				Distance_TIM1_CH4_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4(CAPTURE_TIM1)-Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC4PolarityConfig(CAPTURE_TIM1, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				Distance_TIM1_CH4_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (CAPTURE_TIM1,TIM_IT_CC4);	
		}		
	}

	TIM_ClearITPendingBit (CAPTURE_TIM1,TIM_IT_CC4);	
}

/**************************************************************************
Function: CAPTURE_TIM1_UP_IRQHandler
Input   : none
Output  : none
�������ܣ��ߵ�ƽ�����жϺ���
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	

//������ģ��4�����ж�
void CAPTURE_TIM1_UP_IRQHandler(void)
{
	if(Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( CAPTURE_TIM1, TIM_IT_Update) != RESET )               
		{	
			Distance_TIM1_CH4_ICUserValueStructure.Capture_Period ++;
			//����ж�
			TIM_ClearITPendingBit (CAPTURE_TIM1,TIM_IT_Update);	    
		}
	}
	
	TIM_ClearITPendingBit (CAPTURE_TIM1,TIM_IT_Update);	

}


#endif


//ʹ�ú�ģң��
#ifdef PWM_Capture
/**************************************************************************
Function: PWM_TIM2_IRQHandler
Input   : none
Output  : none
�������ܣ��ߵ�ƽ�����жϺ���
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void PWM_TIM2_IRQHandler(void)
{
	/*************************************ͨ��3*******************************************/
	if(PWM_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( PWM_TIM2, TIM_IT_Update) != RESET )               
			PWM_TIM2_CH3_ICUserValueStructure.Capture_Period ++;
			
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (PWM_TIM2, TIM_IT_CC3 ) != RESET)
		{
			// ��һ�β���
			if ( PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture3 (PWM_TIM2);
				// �Զ���װ�ؼĴ������±�־��0
				PWM_TIM2_CH3_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC3PolarityConfig(PWM_TIM2, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture3 (PWM_TIM2)-PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC3PolarityConfig(PWM_TIM2, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				PWM_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (PWM_TIM2,TIM_IT_CC3);	
		}		
	}
	/*************************************ͨ��4*******************************************/
	if(PWM_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( PWM_TIM2, TIM_IT_Update) != RESET )               
			PWM_TIM2_CH4_ICUserValueStructure.Capture_Period ++;
			
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (PWM_TIM2, TIM_IT_CC4 ) != RESET)
		{
			// ��һ�β���
			if ( PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4 (PWM_TIM2);
				// �Զ���װ�ؼĴ������±�־��0
				PWM_TIM2_CH4_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC4PolarityConfig(PWM_TIM2, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4(PWM_TIM2)-PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC4PolarityConfig(PWM_TIM2, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				PWM_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (PWM_TIM2,TIM_IT_CC4);	
		}		
	}
	TIM_ClearITPendingBit (CAPTURE_TIM2,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4);	

}

/**************************************************************************
Function: PWM_TIM1_CC_IRQHandler
Input   : none
Output  : none
�������ܣ��ߵ�ƽ�����жϺ���
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	

void PWM_TIM1_CC_IRQHandler(void)
{
		/*************************************ͨ��1*******************************************/
	if(PWM_TIM1_CH1_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (PWM_TIM1, TIM_IT_CC1 ) != RESET)
		{
			// ��һ�β���
			if ( PWM_TIM1_CH1_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				PWM_TIM1_CH1_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture1 (PWM_TIM1);
				// �Զ���װ�ؼĴ������±�־��0
				PWM_TIM1_CH1_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC1PolarityConfig(PWM_TIM1, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				PWM_TIM1_CH1_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				PWM_TIM1_CH1_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture1(PWM_TIM1)-PWM_TIM1_CH1_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC1PolarityConfig(PWM_TIM1, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				PWM_TIM1_CH1_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				PWM_TIM1_CH1_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (PWM_TIM1,TIM_IT_CC1);	
		}		
	}

	
	/*************************************ͨ��4*******************************************/
	if(PWM_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		// �����жϣ���һ�����������жϣ��ڶ������½����ж�
		if ( TIM_GetITStatus (PWM_TIM1, TIM_IT_CC4 ) != RESET)
		{
			// ��һ�β���
			if ( PWM_TIM1_CH4_ICUserValueStructure.Capture_StartFlag == 0 )
			{
				//��һ�β���ʱ���Ѳ���ֵ��������
				PWM_TIM1_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4 (PWM_TIM1);
				// �Զ���װ�ؼĴ������±�־��0
				PWM_TIM1_CH4_ICUserValueStructure.Capture_Period = 0;
//				// �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
//				Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 0;

				// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
				TIM_OC4PolarityConfig(PWM_TIM1, CAPTURE_TIM_END_ICPolarity);
				// ��ʼ�����־λ��1			
				PWM_TIM1_CH4_ICUserValueStructure.Capture_StartFlag = 1;			
			}
			// �½��ز����ж�
			else // �ڶ��β���
			{
				// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
				PWM_TIM1_CH4_ICUserValueStructure.Capture_CcrValue = 
				TIM_GetCapture4(PWM_TIM1)-PWM_TIM1_CH4_ICUserValueStructure.Capture_CcrValue;

				// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
				TIM_OC4PolarityConfig(PWM_TIM1, CAPTURE_TIM_STRAT_ICPolarity);
				// ��ʼ�����־��0		
				PWM_TIM1_CH4_ICUserValueStructure.Capture_StartFlag = 0;
				// ������ɱ�־��1			
				PWM_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag = 1;		
			}
			//����ж�
			TIM_ClearITPendingBit (PWM_TIM1,TIM_IT_CC4);	
		}		
	}

	TIM_ClearITPendingBit (CAPTURE_TIM1,TIM_IT_CC4|TIM_IT_CC1);	
}

/**************************************************************************
Function: PWM_TIM1_UP_IRQHandler
Input   : none
Output  : none
�������ܣ��ߵ�ƽ�����жϺ���
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	

void PWM_TIM1_UP_IRQHandler(void)
{
	//ͨ��4
	if(PWM_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( PWM_TIM1, TIM_IT_Update) != RESET )               
			PWM_TIM1_CH4_ICUserValueStructure.Capture_Period ++;
	}
	//ͨ��1
	if(PWM_TIM1_CH1_ICUserValueStructure.Capture_FinishFlag == 0)//û�����һ�ε�ʱ�򲶻���ܽ�ȥ����ֹ�����������
	{
		if ( TIM_GetITStatus ( PWM_TIM1, TIM_IT_Update) != RESET )               
			PWM_TIM1_CH1_ICUserValueStructure.Capture_Period ++;
	}

	TIM_ClearITPendingBit (PWM_TIM1,TIM_IT_Update);	

}


#endif




