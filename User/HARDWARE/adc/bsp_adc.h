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

#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"



//��Ƭ����������ѹ3.3V
#define Max_Voltage   				3.3f
//ADC��ȡ�����ֵ4095
#define Max_Voltage_ADC				4095
//��ص�ѹ���ȡ��ѹ�ı�����11:1
#define Ratio 						11
//����ѡ��ADC��ѹ�����3.3/2V 
#define Max_Car_ADC					2047
//һ���ĸ���
#define Num_Of_Car					4


// ADC ���ѡ��
// ������ ADC1/2�����ʹ��ADC3���ж���ص�Ҫ�ĳ�ADC3��

/**************************��ѹ���ADC��ADC2*****************************/
#define    ADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    ADCx                          ADC2
#define    ADC_CLK                       RCC_APB2Periph_ADC2

// ADC GPIO�궨��
// ע�⣺����ADC�ɼ���IO����û�и��ã�����ɼ���ѹ����Ӱ��
#define    ADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define    ADC_GPIO_CLK                  RCC_APB2Periph_GPIOC  
#define    ADC_PORT                      GPIOC
#define    ADC_PIN                       GPIO_Pin_1
// ADC ͨ���궨��
#define    ADC_CHANNEL                   ADC_Channel_11

// ADC �ж���غ궨��
#define    ADC_IRQ                       ADC1_2_IRQn
#define    ADC_IRQHandler                ADC1_2_IRQHandler




/*******************************����ѡ��ADC*************************/
//ADC1
#define    CAR_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
#define    CAR_ADC                          	 ADC1
#define    CAR_ADC_CLK                      	 RCC_APB2Periph_ADC1

#define    CAR_ADC_GPIO_APBxClock_FUN       	 RCC_APB2PeriphClockCmd
#define    CAR_ADC_GPIO_CLK                	  	 RCC_APB2Periph_GPIOC  
#define    CAR_ADC_PORT                     	 GPIOC
#define    CAR_ADC_PIN                       	 GPIO_Pin_0


// ADC ͨ���궨��
#define    CAR_ADC_CHANNEL                  	 ADC_Channel_10
#define    ELE_ADC_L_CHANNEL					 ADC_Channel_4
#define    ELE_ADC_M_CHANNEL					 ADC_Channel_5
#define    ELE_ADC_R_CHANNEL					 ADC_Channel_15
#define    CCD_ADC_CHANNEL					 	 ADC_Channel_15



extern __IO uint16_t ADC_ConvertedValue;



void Voltage_ADC_Init(void);
u16 Get_Voltage(void);
void Car_Select_ADC_Init(void);
u16 Get_Adc(u8 ch);   
void dianweiqi(void);
u16 Get_Adc2(u8 ch);  

#endif /* __ADC_H */

