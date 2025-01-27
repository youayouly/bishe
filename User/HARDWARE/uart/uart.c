
#include "uart.h"

// ����4 uart
void UART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;


	// �򿪴���GPIO��ʱ��
	OPENMV_USART_GPIO_APBxClkCmd(OPENMV_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	OPENMV_USART_APBxClkCmd(OPENMV_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = OPENMV_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OPENMV_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = OPENMV_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(OPENMV_USART_RX_GPIO_PORT, &GPIO_InitStructure);	

	//����USARTΪ�ж�Դ 
	NVIC_InitStructure.NVIC_IRQChannel = OPENMV_USART_IRQ;
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
	USART_InitStructure.USART_BaudRate = OPENMV_USART_BAUDRATE;
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
	USART_Init(OPENMV_USARTx, &USART_InitStructure);

	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(OPENMV_USARTx, USART_IT_RXNE, ENABLE);	

	// ʹ�ܴ���
	USART_Cmd(OPENMV_USARTx, ENABLE);	    

}

void USART_SendByte(uint8_t data)
{
  
      // �ȴ����ͻ�����Ϊ��
    while (USART_GetFlagStatus(OPENMV_USARTx, USART_FLAG_TXE) == RESET);
    // ��������
    USART_SendData(OPENMV_USARTx, data);
  
  
  
}

