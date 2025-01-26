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



#ifndef __ELE_CCD_H
#define	__ELE_CCD_H

#include "stm32f10x.h"
#include "Header.h"

/*******************************���Ѳ��ADC**************************/
//PA4��PA5��PC5--IN4��IN5��IN15

#define    ELE_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
#define    ELE_ADC                          	 ADC1
#define    ELE_ADC_CLK                      	 RCC_APB2Periph_ADC1
#define    ELE_ADC_GPIO_APBxClock_FUN       	 RCC_APB2PeriphClockCmd

//��·���Ѳ�߶˿�A4
#define    ELE_ADC_L_GPIO_CLK                	 RCC_APB2Periph_GPIOA  
#define    ELE_ADC_L_PORT                     	 GPIOA
#define    ELE_ADC_L_PIN                       	 GPIO_Pin_4

//�м���Ѳ�߶˿�A5
#define    ELE_ADC_M_GPIO_CLK                	 RCC_APB2Periph_GPIOA  
#define    ELE_ADC_M_PORT                     	 GPIOA
#define    ELE_ADC_M_PIN                       	 GPIO_Pin_5

//��·���Ѳ�߶˿�C5
#define    ELE_ADC_R_GPIO_CLK                	 RCC_APB2Periph_GPIOC  
#define    ELE_ADC_R_PORT                     	 GPIOC
#define    ELE_ADC_R_PIN                       	 GPIO_Pin_5


/********************************CCDѲ��***************************/
//PA4--TSL_SI��PA5--TSL_CLK;PC5--ADC

//TSL_SI
#define    TSL_SI_GPIO_CLK                	 	RCC_APB2Periph_GPIOA  
#define    TSL_SI_PORT                     	 	GPIOA
#define    TSL_SI_PIN                       	GPIO_Pin_4


//TSL_CLK
#define    TSL_CLK_GPIO_CLK                	 	RCC_APB2Periph_GPIOA  
#define    TSL_CLK_PORT                     	GPIOA
#define    TSL_CLK_PIN                       	GPIO_Pin_5

//CCD_ADC
#define    CCD_ADC_GPIO_CLK                	 	RCC_APB2Periph_GPIOC  
#define    CCD_ADC_PORT                     	GPIOC
#define    CCD_ADC_PIN                       	GPIO_Pin_5

#define    CCD_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
#define    CCD_ADC                          	 ADC1
#define    CCD_ADC_CLK                      	 RCC_APB2Periph_ADC1



#define TSL_SI_HIGH								PAout(4) = 1
#define TSL_SI_LOW								PAout(4) = 0
#define TSL_CLK_HIGH							PAout(5) = 1
#define TSL_CLK_LOW								PAout(5) = 0



#define Barrier_Detected						1
#define No_Barrier								0


extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
extern u16  CCD_ADV[128];
extern u8 CCD_Median,CCD_Threshold;             


void ELE_ADC_Init(void);
void ELE_Mode(void);
void CCD_Init(void);
void Dly_us(void);
void RD_TSL(void);
void  Find_CCD_Median(void);
void CCD_Mode(void);



u8 Detect_Barrier(void);


#endif
