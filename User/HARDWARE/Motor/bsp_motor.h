#ifndef __MOTOR_H
#define	__MOTOR_H


#include "stm32f10x.h"
#include "./GeneralTim/bsp_GeneralTim.h" 

//引脚定义

//#define    MOTOR1_GPIO_CLK     			RCC_APB2Periph_GPIOA
//#define    MOTOR1_GPIO_PORT    			GPIOA			   
//#define    MOTOR1_IN1_GPIO_PIN	 		GPIO_Pin_6
//#define    MOTOR1_IN2_GPIO_PIN	 		GPIO_Pin_7


//#define    MOTOR2_GPIO_CLK     			RCC_APB2Periph_GPIOB
//#define    MOTOR2_GPIO_PORT    			GPIOB			   
//#define    MOTOR2_IN1_GPIO_PIN	 		GPIO_Pin_0
//#define    MOTOR2_IN2_GPIO_PIN	 		GPIO_Pin_1


//引脚定义
//舵机
#define    	SERVO_GPIO_CLK     					RCC_APB2Periph_GPIOA
#define    	SERVO_GPIO_PORT    					GPIOA			   
#define    	SERVO_GPIO_PIN	 					GPIO_Pin_8
//使用定时器TIM1
#define    	SERVO_TIM                   		TIM1
#define    	SERVO_TIM_APBxClock_FUN     		RCC_APB2PeriphClockCmd
#define    	SERVO_TIM_CLK               		RCC_APB2Periph_TIM1
#define    	SERVO_TIM_OCXInit_FUN				TIM_OC1Init
#define    	SERVO_TIM_OCXPreloadConfig_FUN		TIM_OC1PreloadConfig
#define 	SERVO_TIM_SetCompareX_FUN		  	TIM_SetCompare1




//这里是电机输出的PWM
#define PWMA_IN1 TIM3->CCR1
#define PWMA_IN2 TIM3->CCR2
#define PWMB_IN1 TIM3->CCR3
#define PWMB_IN2 TIM3->CCR4


void Motor_Init(u16 arr,u16 psc);
void Set_Pwm(int motor_a,int motor_b);
void Servo_Init(u16 arr,u16 psc);

#endif
