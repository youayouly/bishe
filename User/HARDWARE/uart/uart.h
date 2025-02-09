#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"
#include "Header.h"

// 串口初始化函数
void UART_Init(void);
// 串口发送单个字符
void USART_Send(uint8_t data);
// 串口发送字符串
void USART_SendString(const char* str);
// 解析接收到的数据
void parse_received_data(uint8_t* data);

void OPENMV_USART_IRQHandler(void) ;
// 定义接收缓冲区大小

void USART_SendChecksum_Text(uint8_t checksum);
void process_ring_buffer_frame(void);
  void ProcessReceivedData(void);
void USART_Send_Float(float value);

#define RX_BUF_SIZE 64


// 外部声明全局变量
extern uint8_t rx_buf[RX_BUF_SIZE];  // 接收缓冲区
extern uint8_t rx_index;  // 缓存索引
extern uint8_t data_ready;  // 数据是否准备好

#endif  // __UART_H
