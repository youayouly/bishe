#include "uart.h"

uint8_t rx_buf[RX_BUF_SIZE];  // 定义接收缓冲区
uint8_t rx_index = 0;         // 初始化缓冲区索引
uint8_t data_ready = 0;       // 初始化数据准备标志

void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 打开串口GPIO的时钟
    OPENMV_USART_GPIO_APBxClkCmd(OPENMV_USART_GPIO_CLK, ENABLE);
    
    // 打开串口外设的时钟
    OPENMV_USART_APBxClkCmd(OPENMV_USART_CLK, ENABLE);

    // 将USART Tx的GPIO配置为推挽复用模式
    GPIO_InitStructure.GPIO_Pin = OPENMV_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OPENMV_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // 将USART Rx的GPIO配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = OPENMV_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(OPENMV_USART_RX_GPIO_PORT, &GPIO_InitStructure);  

    // 配置USART为中断源
    NVIC_InitStructure.NVIC_IRQChannel = OPENMV_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 抢断优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // 使能中断
    NVIC_Init(&NVIC_InitStructure);

    // 配置串口的工作参数
    USART_InitStructure.USART_BaudRate = OPENMV_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // 完成串口的初始化配置
    USART_Init(OPENMV_USARTx, &USART_InitStructure);

    // 使能串口接收中断
    USART_ITConfig(OPENMV_USARTx, USART_IT_RXNE, ENABLE);  

    // 使能串口
    USART_Cmd(OPENMV_USARTx, ENABLE);    
}

// 修改为发送 8 位校验和的文本（转换为 2 位十六进制字符串）
void USART_SendChecksum_Text(uint8_t checksum) {
    char buf[3];  // 保存转换后的字符串，格式"XX" + '\0'
    sprintf(buf, "%02X", checksum); // 转换为2位十六进制字符串，例如"89"
    USART_SendString(buf);          // 发送这个字符串
}

void USART_Send(uint8_t data)
{
    // 等待发送缓冲区为空
    while (USART_GetFlagStatus(OPENMV_USARTx, USART_FLAG_TXE) == RESET);
    // 发送数据
    USART_SendData(OPENMV_USARTx, data);
}

void USART_SendString(const char* str)
{
    while (*str) {
        USART_Send((uint8_t)(*str));  // 发送每个字符
        str++;
    }
}

void parse_received_data(uint8_t* data) {
    uint8_t parsed_ball_detected;
    int16_t x, y, distance, angle;

    if (sscanf((char*)data, "$%hhu,%hd,%hd,%hd,%hd", 
               &parsed_ball_detected, &x, &y, &distance, &angle) != 5) {
        USART_SendString("ERR:FMT\n");
        return;
    }

    if (x < 0 || x > 640 || y < 0 || y > 480 || distance < 0) {
        USART_SendString("ERR:RANGE\n");
        return;
    }

    // 如果所有数据均为0，则不发送数据
    if (parsed_ball_detected == 0 && x == 0 && y == 0 && distance == 0 && angle == 0) {
        return;
    }

    // 更新全局变量
    ball_detected = parsed_ball_detected;
    ball_x = x;
    ball_y = y;
    ball_distance = distance;
    ball_angle = angle;

    // 发送 ACK 回复
    USART_SendString("ACK\n");
}

// UART中断处理函数
void OPENMV_USART_IRQHandler(void) {
    if (USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);
        
        // 帧起始检测
        if (ch == '$' && rx_index == 0) {
            rx_index = 0;
            rx_buf[rx_index++] = ch;
        }
        // 数据积累
        else if (rx_index > 0 && rx_index < RX_BUF_SIZE - 1) {
            rx_buf[rx_index++] = ch;
            if (ch == '\n') {  // 检测到帧结束符
                rx_buf[rx_index] = '\0';
                int16_t parsed_type, x, y, dist, angle;
                if (sscanf((char*)rx_buf, "$%hd,%hd,%hd,%hd,%hd", 
                           &parsed_type, &x, &y, &dist, &angle) == 5) {
                    // 若所有数据均为0，则不发送回复
                    if (!(parsed_type == 0 && x == 0 && y == 0 && dist == 0 && angle == 0)) {
                        ball_detected = parsed_type;
                        ball_x = x;
                        ball_y = y;
                        ball_distance = dist;
                        ball_angle = angle;
                        USART_SendString("ACK\n");
                    }
                }
                rx_index = 0;
            }
        }
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}
