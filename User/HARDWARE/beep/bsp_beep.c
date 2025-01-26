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
  
#include "./beep/bsp_beep.h"   

/**************************************************************************
Function: Buzzer initialization
Input   : none
Output  : none
�������ܣ���������ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void BEEP_GPIO_Config(void)
{		

	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*�ر�JTAG�ӿ�*/
	JTAG_Set(JTAG_SWD_DISABLE);    

	/*��SWD�ӿ� �������������SWD�ӿڵ���*/
	JTAG_Set(SWD_ENABLE);           

	/*�������Ʒ�������GPIO�Ķ˿�ʱ��*/
	RCC_APB2PeriphClockCmd( BEEP_GPIO_CLK, ENABLE); 

	/*ѡ��Ҫ���Ʒ�������GPIO*/															   
	GPIO_InitStructure.GPIO_Pin = BEEP_GPIO_PIN;	

	/*����GPIOģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*����GPIO����Ϊ50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

	/*���ÿ⺯������ʼ�����Ʒ�������GPIO*/
	GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStructure);			 

	/* �رշ�����*/
	GPIO_ResetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);	 
}


/**************************************************************************
Function: Set JTAG mode
Input   : mode:JTAG, swd mode settings��00��all enable��01��enable SWD��10��Full shutdown
Output  : none
�������ܣ�����JTAGģʽ
��ڲ�����mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;	
����  ֵ����
**************************************************************************/
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00	
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //����jtagģʽ
} 



/**************************************************************************
Function: Buzzer_Alarm
Input   : Indicates the count of frequencies
Output  : none
�������ܣ�����������
��ڲ���: ָʾƵ�ʵļ��� 
����  ֵ����
**************************************************************************/	 	
//���жϺ�������
void Buzzer_Alarm(u16 count)
{
	static int count_time;
	if(0 == count)
	{
		BEEP_OFF;
	}
	else if(++count_time >= count)
	{
		BEEP_TOGGLE;
		count_time = 0;	
	}
}




/*********************************************END OF FILE**********************/
