#ifndef __BSP_GENERALTIME_H
#define __BSP_GENERALTIME_H


#include "stm32f10x.h"
#include "Header.h"

/************ͨ�ö�ʱ��TIM�������壬ֻ��TIM2��3��4��5************/
// ��ʹ�ò�ͬ�Ķ�ʱ����ʱ�򣬶�Ӧ��GPIO�ǲ�һ���ģ����Ҫע��
//ֻ���޸ĺ궨��
//�������PWM
//ʹ�ö�ʱ��TIM3
#define            PWM_OutPut_TIM                   TIM3
#define            PWM_OutPut_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            PWM_OutPut_TIM_CLK               RCC_APB1Periph_TIM3
//#define            GENERAL_TIM_Period            9
//#define            GENERAL_TIM_Prescaler         71
// TIM3 ����Ƚ�ͨ��1
#define            PWM_OutPut_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            PWM_OutPut_TIM_CH1_PORT          GPIOA
#define            PWM_OutPut_TIM_CH1_PIN           GPIO_Pin_6

// TIM3 ����Ƚ�ͨ��2
#define            PWM_OutPut_TIM_CH2_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            PWM_OutPut_TIM_CH2_PORT          GPIOA
#define            PWM_OutPut_TIM_CH2_PIN           GPIO_Pin_7

// TIM3 ����Ƚ�ͨ��3
#define            PWM_OutPut_TIM_CH3_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_OutPut_TIM_CH3_PORT          GPIOB
#define            PWM_OutPut_TIM_CH3_PIN           GPIO_Pin_0

// TIM3 ����Ƚ�ͨ��4
#define            PWM_OutPut_TIM_CH4_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_OutPut_TIM_CH4_PORT          GPIOB
#define            PWM_OutPut_TIM_CH4_PIN           GPIO_Pin_1



//����ʹ�ö�ʱ����
//ʹ�ö�ʱ��TIM5
#define 			TIMING_TIM					 	TIM5
#define            	TIMING_TIM_APBxClock_FUN     	RCC_APB1PeriphClockCmd
#define            	TIMING_TIM_CLK               	RCC_APB1Periph_TIM5
#define            	TIMING_TIM_IRQ               	TIM5_IRQn
#define           	TIMING_TIM_IRQHandler        	TIM5_IRQHandler



/***************************��������*******************************/

void PWM_OutPut_TIM_Init(u16 arr,u16 psc);
void TIMING_TIM_Init(u16 arr,u16 psc);

#endif	/* __BSP_GENERALTIME_H */


