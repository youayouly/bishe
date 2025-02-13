#ifndef __SERVO_H
#define __SERVO_H 	   

#include "stm32f10x.h"
#include "Header.h"
extern int SERVO3_INIT;	

// ����ź�����ʹ��PA3
#define    	SERVO3_GPIO_CLK     					RCC_APB2Periph_GPIOA
#define    	SERVO3_GPIO_PORT    					GPIOA			   
#define    	SERVO3_GPIO_PIN	 					GPIO_Pin_3  // ʹ��PA3

// ʹ�ö�ʱ��TIM2������TIM2��CH4��ӦPA3��
#define    	SERVO3_TIM                   		TIM2
#define    	SERVO3_TIM_APBxClock_FUN     		RCC_APB1PeriphClockCmd
#define    	SERVO3_TIM_CLK               		RCC_APB1Periph_TIM2
#define    	SERVO3_TIM_OCXInit_FUN				TIM_OC4Init  // TIM2��ͨ��4
#define    	SERVO3_TIM_OCXPreloadConfig_FUN		TIM_OC4PreloadConfig
#define 	SERVO3_TIM_SetCompareX_FUN		  	TIM_SetCompare4
void Servo_Init3(u16 arr,u16 psc);
void Set_Servo_PWM(uint16_t pwm_value);
#endif
