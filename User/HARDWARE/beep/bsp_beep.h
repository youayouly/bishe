#ifndef __BEEP_H
#define	__BEEP_H


#include "stm32f10x.h"
#include "Header.h"


/*JTAGģʽ���ö���*/
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	


/* ������������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����Ĵ��뼴�ɸı���Ƶķ��������� */
#define BEEP_GPIO_PORT    	GPIOA			              	/* GPIO�˿� */
#define BEEP_GPIO_CLK 	    RCC_APB2Periph_GPIOA			/* GPIO�˿�ʱ�� */
#define BEEP_GPIO_PIN		GPIO_Pin_15			        	/* ���ӵ���������GPIO */


/* ���κ꣬��������������һ��ʹ�� */
#define BEEP(a)	if (a)	\
					GPIO_SetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN);\
					else		\
					GPIO_ResetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN)

					
/* �������IO�ĺ� */
#define BEEP_TOGGLE		    digitalToggle(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_ON		   		digitalHi(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_OFF			digitalLo(BEEP_GPIO_PORT,BEEP_GPIO_PIN)


					
void BEEP_GPIO_Config(void);
void JTAG_Set(u8 mode);
void Buzzer_Alarm(u16 count);

					
#endif /* __BEEP_H */
