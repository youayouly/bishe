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


#ifndef __SHOW_H
#define	__SHOW_H

#include "stm32f10x.h"
#include "./OLED/bsp_oled.h"


#define CHANNEL_NUMBER    3				//ʾ����ͨ���������������ʾͨ����


void Show(void);
void Car_Select_Show(void);

u16 myabs(int Input);
void DataScope(void);
void APP_Show(void);
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t);
void OLED_Show_CCD(void);
void Robot_Select(void);


#endif
