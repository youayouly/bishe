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

#define RX_BUF_SIZE 64


// �ⲿ����ȫ�ֱ���
extern uint8_t rx_buf[RX_BUF_SIZE];  // ���ջ�����
extern uint8_t rx_index;  // ��������
extern uint8_t data_ready;  // �����Ƿ�׼����


extern uint8_t pkt_cnt ;
extern uint8_t state ;

#endif  // __UART_H
