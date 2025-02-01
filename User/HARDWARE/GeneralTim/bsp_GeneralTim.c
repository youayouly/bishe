#include "./GeneralTim/bsp_GeneralTim.h" 

/**************************************************************************
Function: PWM_OutPut_TIM_GPIO_Config
Input   : none
Output  : none
�������ܣ�����PWM����˿ڣ����ڿ��Ƶ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
static void PWM_OutPut_TIM_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// ����Ƚ�ͨ��1 GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(PWM_OutPut_TIM_CH1_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  PWM_OutPut_TIM_CH1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_OutPut_TIM_CH1_PORT, &GPIO_InitStructure);

	// ����Ƚ�ͨ��2 GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(PWM_OutPut_TIM_CH2_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  PWM_OutPut_TIM_CH2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_OutPut_TIM_CH2_PORT, &GPIO_InitStructure);

	// ����Ƚ�ͨ��3 GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(PWM_OutPut_TIM_CH3_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  PWM_OutPut_TIM_CH3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_OutPut_TIM_CH3_PORT, &GPIO_InitStructure);

	// ����Ƚ�ͨ��4 GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(PWM_OutPut_TIM_CH4_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  PWM_OutPut_TIM_CH4_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_OutPut_TIM_CH4_PORT, &GPIO_InitStructure);
}


///*
// * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
// * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
// * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            ����
// *	TIM_CounterMode			     TIMx,x[6,7]û�У���������
// *  TIM_Period               ����
// *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
// *}TIM_TimeBaseInitTypeDef; 
// *-----------------------------------------------------------------------------
// */

/* ----------------   PWM�ź� ���ں�ռ�ձȵļ���--------------- */
// ARR ���Զ���װ�ؼĴ�����ֵ
// CLK_cnt����������ʱ�ӣ����� Fck_int / (psc+1) = 72M/(psc+1)
// PWM �źŵ����� T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// ռ�ձ�P=CCR/(ARR+1)

/**************************************************************************
Function: PWM_OutPut_TIM_Mode_Config
Input   : none
Output  : none
�������ܣ�����PWM���ģʽ�����ڿ��Ƶ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
static void PWM_OutPut_TIM_Mode_Config(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

  // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	PWM_OutPut_TIM_APBxClock_FUN(PWM_OutPut_TIM_CLK,ENABLE);

	/*--------------------ʱ���ṹ���ʼ��-------------------------*/

	TIM_TimeBaseInitStruct.TIM_Period = arr;              			//�趨�������Զ���װֵ 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          			//�趨Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        //����ʱ�ӷָ�
	TIM_TimeBaseInit(PWM_OutPut_TIM,&TIM_TimeBaseInitStruct);      	//��ʼ����ʱ��

	
	/*--------------------����ȽϽṹ���ʼ��-------------------*/	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//�����������
	TIM_OC1Init(PWM_OutPut_TIM,&TIM_OCInitStruct);                	//��ʼ������Ƚϲ���

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//�����������
	TIM_OC2Init(PWM_OutPut_TIM,&TIM_OCInitStruct);                 	//��ʼ������Ƚϲ���

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//�����������
	TIM_OC3Init(PWM_OutPut_TIM,&TIM_OCInitStruct);                  //��ʼ������Ƚϲ���

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//�����������
	TIM_OC4Init(PWM_OutPut_TIM,&TIM_OCInitStruct);                  //��ʼ������Ƚϲ���

	TIM_OC1PreloadConfig(PWM_OutPut_TIM,TIM_OCPreload_Enable);   	//CH1ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(PWM_OutPut_TIM,TIM_OCPreload_Enable);   	//CH2ʹ��Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(PWM_OutPut_TIM,TIM_OCPreload_Enable);   	//CH3ʹ��Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(PWM_OutPut_TIM,TIM_OCPreload_Enable);   	//CH4ʹ��Ԥװ�ؼĴ���

	TIM_ARRPreloadConfig(PWM_OutPut_TIM, ENABLE);                	//ʹPWM_OutPut_TIM(��ʱ��3)��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(PWM_OutPut_TIM,ENABLE);                              	//ʹ�ܶ�ʱ��3
}
/**************************************************************************
Function: PWM_OutPut_TIM_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ���ʼ��PWM��������ڿ��Ƶ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void PWM_OutPut_TIM_Init(u16 arr,u16 psc)
{
	PWM_OutPut_TIM_GPIO_Config();			//GPIO����
	PWM_OutPut_TIM_Mode_Config(arr,psc);	//ģʽ����
}

/**************************************************************************
Function: TIMING_TIM_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ���ʱ�жϳ�ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
//��ʱ��TIM5
void TIMING_TIM_Init(u16 arr,u16 psc)
{
	//��ʱʱ�䣺(arr+1)*(psc+1)/CLK
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure; 
		// �����ж���Ϊ0
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = TIMING_TIM_IRQ ;	
		// ���������ȼ�Ϊ 1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	 
		// ������ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	TIMING_TIM_APBxClock_FUN(TIMING_TIM_CLK,ENABLE);

	/*--------------------ʱ���ṹ���ʼ��-------------------------*/
	TIM_TimeBaseInitStruct.TIM_Period = arr;              			//�趨�������Զ���װֵ 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          			//�趨Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        //����ʱ�ӷָ�
	TIM_TimeBaseInit(TIMING_TIM,&TIM_TimeBaseInitStruct);      		//��ʼ����ʱ��
		// ����������жϱ�־λ
    TIM_ClearFlag(TIMING_TIM, TIM_FLAG_Update);
		// �����������ж�
    TIM_ITConfig(TIMING_TIM,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIMING_TIM,ENABLE);                              		//ʹ�ܶ�ʱ��

}


/***********************************END OF FILE********************************/
