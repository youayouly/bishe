#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"
#include "Header.h"

// ���ڳ�ʼ������
void UART_Init(void);
// ���ڷ��͵����ַ�
void USART_Send(uint8_t data);
// ���ڷ����ַ���
void USART_SendString(const char* str);
// �������յ�������
void parse_received_data(uint8_t* data);

void OPENMV_USART_IRQHandler(void) ;
// ������ջ�������С

void USART_SendChecksum_Text(uint8_t checksum);
void process_ring_buffer_frame(void);
  void ProcessReceivedData(void);
void USART_Send_Float(float value);

#define RX_BUF_SIZE 64


// �ⲿ����ȫ�ֱ���
extern uint8_t rx_buf[RX_BUF_SIZE];  // ���ջ�����
extern uint8_t rx_index;  // ��������
extern uint8_t data_ready;  // �����Ƿ�׼����

#endif  // __UART_H
