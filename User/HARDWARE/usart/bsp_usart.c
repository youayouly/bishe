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
	
#include "./usart/bsp_usart.h"


/**************************************************************************
Function: DEBUG_USART_Init
Input   : none
Output  : none
�������ܣ����Դ��ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/
//����1
void DEBUG_USART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);


	// ����USARTΪ�ж�Դ 
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж� 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC 
	NVIC_Init(&NVIC_InitStructure);
	
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	

	// ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);	    

}


/**************************************************************************
Function: BLUETOOTH_USART_Init
Input   : none
Output  : none
�������ܣ��������ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//����3
void BLUETOOTH_USART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;


	// �򿪴���GPIO��ʱ��
	BLUETOOTH_USART_GPIO_APBxClkCmd(BLUETOOTH_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	BLUETOOTH_USART_APBxClkCmd(BLUETOOTH_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = BLUETOOTH_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLUETOOTH_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = BLUETOOTH_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BLUETOOTH_USART_RX_GPIO_PORT, &GPIO_InitStructure);	

	// ����USARTΪ�ж�Դ 
	NVIC_InitStructure.NVIC_IRQChannel = BLUETOOTH_USART_IRQ;
	//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);



	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = BLUETOOTH_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(BLUETOOTH_USARTx, &USART_InitStructure);

	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(BLUETOOTH_USARTx, USART_IT_RXNE, ENABLE);	

	// ʹ�ܴ���
	USART_Cmd(BLUETOOTH_USARTx, ENABLE);	    

}
/**************************************************************************
Function: LIDAR_USART_Init
Input   : none
Output  : none
�������ܣ��״ﴮ�ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//����5
void LIDAR_USART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;


	// �򿪴���GPIO��ʱ��
	LIDAR_USART_GPIO_APBxClkCmd(LIDAR_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	LIDAR_USART_APBxClkCmd(LIDAR_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = LIDAR_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LIDAR_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = LIDAR_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIDAR_USART_RX_GPIO_PORT, &GPIO_InitStructure);	

	//����USARTΪ�ж�Դ 
	NVIC_InitStructure.NVIC_IRQChannel = LIDAR_USART_IRQ;
	//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж� 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);


	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = LIDAR_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(LIDAR_USARTx, &USART_InitStructure);

	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(LIDAR_USARTx, USART_IT_RXNE, ENABLE);	

	// ʹ�ܴ���
	USART_Cmd(LIDAR_USARTx, ENABLE);	    

}
/**************************************************************************
Function: fputc
Input   : none
Output  : none
�������ܣ��ض���c�⺯��printf������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
	if(Flag_Show==0)			//���������1
	{
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		

		/* ����һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		return (ch);
	}
	else					//ʹ��printf���������3
	{
		
		while (USART_GetFlagStatus(BLUETOOTH_USARTx, USART_FLAG_TXE) == RESET);		

		USART_SendData(BLUETOOTH_USARTx, (uint8_t) ch);
	
		return (ch);
		
	}
	
}


///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}

