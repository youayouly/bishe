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

#include "./Motor/bsp_motor.h"   

/**************************************************************************
Function: Motor_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ����Ƶ����PWM��ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void Motor_Init(u16 arr,u16 psc)
{
	PWM_OutPut_TIM_Init(arr,psc);//��ʼ����ʱ��
}



/**************************************************************************
Function: Servo_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ����pwm�����ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void Servo_Init(u16 arr,u16 psc)
{	
	GPIO_InitTypeDef GPIO_InitStruct;                             
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	SERVO_TIM_APBxClock_FUN(SERVO_TIM_CLK,ENABLE);   	//ʹ�ܶ�ʱ��1ʱ��
	RCC_APB2PeriphClockCmd(SERVO_GPIO_CLK,ENABLE);  	//ʹ��GPIOA��ʱ��

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        //�����������
	GPIO_InitStruct.GPIO_Pin = SERVO_GPIO_PIN;          //PA8
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_GPIO_PORT,&GPIO_InitStruct);

	TIM_TimeBaseInitStruct.TIM_Period = arr;              //�趨�������Զ���װֵ 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //�趨Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;    //����ʱ�ӷָ�
	TIM_TimeBaseInit(SERVO_TIM,&TIM_TimeBaseInitStruct);       	//��ʼ����ʱ��

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             	//ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState  = TIM_OutputState_Enable; //�Ƚ����ʹ��
//	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = 0;                            	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     	//�����������
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	
	SERVO_TIM_OCXInit_FUN(SERVO_TIM,&TIM_OCInitStruct);         //��ʼ������Ƚϲ�����ͨ��1

	SERVO_TIM_OCXPreloadConfig_FUN(SERVO_TIM,TIM_OCPreload_Enable);  	//CH1ʹ��Ԥװ�ؼĴ���

	TIM_CtrlPWMOutputs(SERVO_TIM,ENABLE);  							 	//�߼���ʱ�������Ҫ������䣬ͨ�ö�ʱ����ȥ��

	TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);               			 	//ʹ��TIM1��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(SERVO_TIM,ENABLE);                              			//ʹ�ܶ�ʱ��1

		TIM1->CCR1=Servo_PWM;

}

/**************************************************************************
Function: Drive_Motor
Input   : motor_a,motor_b:Indicates the left/right motor PWM duty cycle
Output  : none
�������ܣ��������
��ڲ���: motor_a��motor_b��ֵָʾ���ҵ��PWMռ�ձȣ�ֵԽ���ٶ�Խ��
����  ֵ����
**************************************************************************/	 	
void Set_Pwm(int motor_a,int motor_b)
{
	
	if(Car_Num==Akm_Car)//����������Ҫ���ƶ��
		SERVO_TIM_SetCompareX_FUN(SERVO_TIM,Servo_PWM);
	if(motor_a<0)		PWMA_IN1=7200,PWMA_IN2=7200+motor_a;
	else 	            PWMA_IN2=7200,PWMA_IN1=7200-motor_a;

	if(motor_b<0)		PWMB_IN1=7200,PWMB_IN2=7200+motor_b;
	else 	            PWMB_IN2=7200,PWMB_IN1=7200-motor_b;
}


/************************************END OF FILE*******************************/







