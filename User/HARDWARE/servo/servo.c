#include "servo.h"   

int SERVO3_INIT=1800;
  /**************************************************************************
Function: Servo_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
�������ܣ����pwm�����ʼ��
��ڲ���: Ԥװ��ֵ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 	
void Servo_Init3(u16 arr,u16 psc)
{

	    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // ʹ��GPIOʱ�ӣ�PA3��
    SERVO3_TIM_APBxClock_FUN(SERVO3_GPIO_CLK, ENABLE);

    // ����PA3Ϊ�����������������PWM�źţ�
    GPIO_InitStruct.GPIO_Pin = SERVO3_GPIO_PIN;  // PA3
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // �����������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // ����ٶ�50MHz
    GPIO_Init(SERVO3_GPIO_PORT, &GPIO_InitStruct);

    // ʹ��TIM2ʱ��
    SERVO3_TIM_APBxClock_FUN(SERVO3_TIM_CLK, ENABLE);

    // ���ö�ʱ��TIM2
    TIM_TimeBaseInitStruct.TIM_Period = arr;  // �Զ���װֵ
    TIM_TimeBaseInitStruct.TIM_Prescaler = psc;  // Ԥ��Ƶ��
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  // ���ϼ���ģʽ
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;  // ʱ�ӷ�Ƶ
    TIM_TimeBaseInit(SERVO3_TIM, &TIM_TimeBaseInitStruct);

    // ����PWMģʽ��TIM2ͨ��4��
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;  // PWMģʽ1
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  // ʹ�����
    TIM_OCInitStruct.TIM_Pulse = 0;  // ��ʼռ�ձ�
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;  // �ߵ�ƽ��Ч
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;  // ����״̬Ϊ��λ
    SERVO3_TIM_OCXInit_FUN(SERVO3_TIM, &TIM_OCInitStruct);  // ��ʼ��TIM2ͨ��4

    // ʹ��TIM2ͨ��4��Ԥװ�ؼĴ���
    SERVO3_TIM_OCXPreloadConfig_FUN(SERVO3_TIM, TIM_OCPreload_Enable);

    // ʹ��TIM2��ARRԤװ�ؼĴ���
    TIM_ARRPreloadConfig(SERVO3_TIM, ENABLE);

    // ʹ��TIM2
    TIM_Cmd(SERVO3_TIM, ENABLE);

    // ���ó�ʼռ�ձ�
    SERVO3_TIM_SetCompareX_FUN(SERVO3_TIM, SERVO3_INIT);  // ����TIM2ͨ��4�Ĳ���ȽϼĴ���ֵ
}

void Set_Servo_PWM(uint16_t pwm_value)
{
    // ֱ�ӵ����� Servo_Init3 ���õ��ĺ꣬
    // ���ڸ��¶�ʱ���Ĳ���ȽϼĴ������Ӷ����� PWM ռ�ձ�
    SERVO3_TIM_SetCompareX_FUN(SERVO3_TIM, pwm_value);
}
