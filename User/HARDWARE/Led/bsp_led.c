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
  
#include "./Led/bsp_led.h"   

/**************************************************************************
Function: LED_GPIO_Config
Input   : none
Output  : none
�������ܣ�LED����
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����LED��ص�GPIO����ʱ��*/
	RCC_APB2PeriphClockCmd( LED1_GPIO_CLK | LED2_GPIO_CLK , ENABLE);
	/*ѡ��Ҫ���Ƶ�GPIO����*/
	GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;	

	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

	/*���ÿ⺯������ʼ��GPIO*/
	GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	

	/*ѡ��Ҫ���Ƶ�GPIO����*/
	GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;

	/*���ÿ⺯������ʼ��GPIO*/
	GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);


	/* �ر�����led��	*/
	GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);

	/* �ر�����led��	*/
	GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);	 
    
}

/**************************************************************************
Function: LED_Flash
Input   : Indicates the count of frequencies
Output  : none
�������ܣ�LED��˸Ƶ��
��ڲ���: ָʾƵ�ʵļ��� 
����  ֵ����
**************************************************************************/	 	
//���жϺ�������
void LED_Flash(u16 count)
{
	static int count_time;
	if(0 == count)
	{
		LED1_OFF;
	}
	else if(++count_time >= count)
	{
		LED1_TOGGLE;
		count_time = 0;	
	}
}

/*********************************************END OF FILE**********************/
