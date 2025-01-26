#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"


#define delay_ms  	SysTick_Delay_Ms
#define delay_us   	SysTick_Delay_Us

#define Delay_ms(x) Delay_us(100*x)	 //��λms


void SysTick_Init(void);
void Delay_us(__IO u32 nTime);

void SysTick_Delay_Us( __IO uint32_t us);
void SysTick_Delay_Ms( __IO uint32_t ms);


#endif /* __SYSTICK_H */
