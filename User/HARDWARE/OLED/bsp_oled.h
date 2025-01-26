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

#ifndef __OLED_H
#define __OLED_H			  	 
#include "Header.h"



/* ����OLED���ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����Ĵ��뼴�ɸı���Ƶ�OLED���� */
#define OLED_SCLK_GPIO_PORT    	GPIOC			              /* GPIO�˿� */
#define OLED_SCLK_GPIO_CLK 	    RCC_APB2Periph_GPIOC		/* GPIO�˿�ʱ�� */
#define OLED_SCLK_GPIO_PIN		GPIO_Pin_14			        /* ��Ӧ���ź� */

#define OLED_SDIN_GPIO_PORT    	GPIOB			              /* GPIO�˿� */
#define OLED_SDIN_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define OLED_SDIN_GPIO_PIN		GPIO_Pin_5			        /* ��Ӧ���ź� */

#define OLED_RST_GPIO_PORT    	GPIOB			              /* GPIO�˿� */
#define OLED_RST_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define OLED_RST_GPIO_PIN		GPIO_Pin_4			        /* ��Ӧ���ź� */

#define OLED_RS_GPIO_PORT    	GPIOB			              /* GPIO�˿� */
#define OLED_RS_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define OLED_RS_GPIO_PIN		GPIO_Pin_3			        /* ��Ӧ���ź� */




//-----------------OLED�˿ڶ���---------------- 
#define OLED_RST_Clr() PBout(4)=0   //RST
#define OLED_RST_Set() PBout(4)=1   //RST

#define OLED_RS_Clr() PBout(3)=0    //DC
#define OLED_RS_Set() PBout(3)=1    //DC

#define OLED_SCLK_Clr()  PCout(14)=0  //SCL
#define OLED_SCLK_Set()  PCout(14)=1   //SCL

#define OLED_SDIN_Clr()  PBout(5)=0   //SDA
#define OLED_SDIN_Set()  PBout(5)=1   //SDA

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����





extern u8 OLED_GRAM[128][8];	 

//OLED�����ú���
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	
void OLED_Refresh_GRAM_Line(u8 line);

#endif  
	 
