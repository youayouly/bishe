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

#ifndef __CAPTURE_H
#define	__CAPTURE_H



/*-------------������������ʹ��-----------
��������ʼ��֮����control.c���жϷ������������ֱ��ͨ��Read_Distane������ȡ
�������еı�������
Distance_1,Distance_2,Distance_3,Distance_4;//��������ر��� 
-----------������������ʹ��-----------*/

/*-------------��ģң�س���ʹ�úͽ���-----------
��������Ǽ���PWM�ĸߵ�ƽʱ�䣬������Ҫ��ʹ�����ָ����Ա���趨
-------------��ģң�س���ʹ�úͽ���-----------*/

#include "stm32f10x.h"
#include "control.h"





//�������ͺ�ģң�صĺ궨�壬ֻ��ʹ����һ
//ѡ����Ҫʹ�õ�ע�͵�
#define Distance_Capture
//#define PWM_Capture


//��������������
//ʹ�ö˿�PC15��������ͨIO��,������ģ�鲶���Ƕ�ʱ��2ͨ��2//������ģ��1
#define 			CAPTURE_TRIG_GPIO_CLK1			RCC_APB2Periph_GPIOC
#define            	CAPTURE_TRIG_PORT1         		GPIOC
#define            	CAPTURE_TRIG_PIN1           	GPIO_Pin_15

#define 			TRIG_HIGH1						PCout(15) = 1
#define 			TRIG_LOW1						PCout(15) = 0

//ʹ�ö˿�PA12��������ͨIO��,������ģ�鲶���Ƕ�ʱ��2ͨ��3//������ģ��2
#define 			CAPTURE_TRIG_GPIO_CLK2			RCC_APB2Periph_GPIOA
#define            	CAPTURE_TRIG_PORT2          	GPIOA
#define            	CAPTURE_TRIG_PIN2          		GPIO_Pin_12

#define 			TRIG_HIGH2						PAout(12) = 1
#define 			TRIG_LOW2						PAout(12) = 0

//ʹ�ö˿�PB13��������ͨIO��,������ģ�鲶���Ƕ�ʱ��2ͨ��4//������ģ��3
#define 			CAPTURE_TRIG_GPIO_CLK3			RCC_APB2Periph_GPIOB
#define            	CAPTURE_TRIG_PORT3          	GPIOB
#define            	CAPTURE_TRIG_PIN3          		GPIO_Pin_13

#define 			TRIG_HIGH3						PBout(13) = 1
#define 			TRIG_LOW3						PBout(13) = 0

//ʹ�ö˿�PB12��������ͨIO��,������ģ�鲶���Ƕ�ʱ��1ͨ��4//������ģ��4
#define 			CAPTURE_TRIG_GPIO_CLK4			RCC_APB2Periph_GPIOB
#define            	CAPTURE_TRIG_PORT4         		GPIOB
#define            	CAPTURE_TRIG_PIN4          		GPIO_Pin_12

#define 			TRIG_HIGH4						PBout(12) = 1
#define 			TRIG_LOW4						PBout(12) = 0

//ʹ�øߵ�ƽ������
//��ʱ��TIM2��CH2,CH3,Ch4
#define 			CAPTURE_TIM2_CH2_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH2_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH2_PIN           	GPIO_Pin_1

#define 			CAPTURE_TIM2_CH3_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH3_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH3_PIN           	GPIO_Pin_2

#define 			CAPTURE_TIM2_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH4_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH4_PIN           	GPIO_Pin_3


#define 			CAPTURE_TIM2					TIM2
#define            	CAPTURE_TIM2_APBxClock_FUN     	RCC_APB1PeriphClockCmd
#define            	CAPTURE_TIM2_CLK               	RCC_APB1Periph_TIM2
#define            	CAPTURE_TIM2_IRQ               	TIM2_IRQn
#define           	CAPTURE_TIM2_IRQHandler       	TIM2_IRQHandler
//#define 			CAPTURE_TIM2_CHx					TIM_Channel_2
//#define 			CAPTURE_TIM_IT_CCX 				TIM_IT_CC2//����ͨ��

//ʹ��TIM1_CH4
#define 			CAPTURE_TIM1_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM1_CH4_PORT          	GPIOA
#define            	CAPTURE_TIM1_CH4_PIN           	GPIO_Pin_11


#define 			CAPTURE_TIM1					TIM1
#define            	CAPTURE_TIM1_APBxClock_FUN     	RCC_APB2PeriphClockCmd
#define            	CAPTURE_TIM1_CLK               	RCC_APB2Periph_TIM1
#define            	CAPTURE_TIM1_CC_IRQn             TIM1_CC_IRQn
#define            	CAPTURE_TIM1_UP_IRQn            TIM1_UP_IRQn
#define           	CAPTURE_TIM1_CC_IRQHandler      TIM1_CC_IRQHandler
#define           	CAPTURE_TIM1_UP_IRQHandler      TIM1_UP_IRQHandler






//��ģң�س�ʼ��
//ʹ��TIM2_CH4,CH3,TIM1_CH4,CH1�ֱ���4·��ģ
//����PWM�ĸߵ�ƽ
#define 			PWM_TIM2_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM2_CH4_PORT          	GPIOA
#define            	PWM_TIM2_CH4_PIN           	GPIO_Pin_3

#define 			PWM_TIM2_CH3_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM2_CH3_PORT          	GPIOA
#define            	PWM_TIM2_CH3_PIN           	GPIO_Pin_2

#define 			PWM_TIM2					TIM2
#define            	PWM_TIM2_APBxClock_FUN     	RCC_APB1PeriphClockCmd
#define            	PWM_TIM2_CLK               	RCC_APB1Periph_TIM2
#define            	PWM_TIM2_IRQ               	TIM2_IRQn
#define           	PWM_TIM2_IRQHandler       	TIM2_IRQHandler

#define 			PWM_TIM1_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM1_CH4_PORT          	GPIOA
#define            	PWM_TIM1_CH4_PIN           	GPIO_Pin_11

#define 			PWM_TIM1_CH1_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	PWM_TIM1_CH1_PORT          	GPIOA
#define            	PWM_TIM1_CH1_PIN           	GPIO_Pin_8

#define 			PWM_TIM1					TIM1
#define            	PWM_TIM1_APBxClock_FUN     	RCC_APB2PeriphClockCmd
#define            	PWM_TIM1_CLK               	RCC_APB2Periph_TIM1
#define            	PWM_TIM1_CC_IRQn             TIM1_CC_IRQn
#define            	PWM_TIM1_UP_IRQn            TIM1_UP_IRQn
#define           	PWM_TIM1_CC_IRQHandler      TIM1_CC_IRQHandler
#define           	PWM_TIM1_UP_IRQHandler      TIM1_UP_IRQHandler



//// ��ȡ����Ĵ���ֵ�����궨��
//#define            CAPTURE_TIM_GetCapturex_FUN                 TIM_GetCapture2
//// �����źż��Ժ����궨��
//#define            CAPTURE_TIM_OCxPolarityConfig_FUN           TIM_OC2PolarityConfig
// ��������ʼ����
#define            CAPTURE_TIM_STRAT_ICPolarity                TIM_ICPolarity_Rising
// �����Ľ�������
#define            CAPTURE_TIM_END_ICPolarity                  TIM_ICPolarity_Falling
//���ٺ궨��
#define 			Light_Speed								   340

// ��ʱ�����벶���û��Զ�������ṹ������
typedef struct
{   
	uint8_t   Capture_FinishFlag;   // ���������־λ
	uint8_t   Capture_StartFlag;    // ����ʼ��־λ
	int  	  Capture_CcrValue;     // ����Ĵ�����ֵ
	uint16_t  Capture_Period;       // �Զ���װ�ؼĴ������±�־ 
}TIM_ICUserValueTypeDef;


extern u16 Distance1,Distance2,Distance3,Distance4;	

void Read_Distane(void);        
void Distance_Cap_Init(u16 arr,u16 psc);
void PWM_Cap_Init(u16 arr,u16 psc);


#endif
