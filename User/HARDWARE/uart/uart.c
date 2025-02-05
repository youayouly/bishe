#include "uart.h"

uint8_t rx_buf[RX_BUF_SIZE];  // 定义接收缓冲区
uint8_t rx_index = 0;  // 初始化缓冲区索引
uint8_t data_ready = 0;  // 初始化数据准备标志



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
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // 使能中断
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
    uint8_t ball_type;
    int16_t x, y, distance, angle;
    uint8_t received_checksum, calculated_checksum = 0;
    
    // 增加错误检测
    if (sscanf((char*)data, "$%hhu,%hd,%hd,%hd,%hd,%hhx", 
              &ball_type, &x, &y, &distance, &angle, &received_checksum) != 6) {
        printf("格式错误: %s\n", data);
        USART_SendString("ERR\n");
        return;
    }

    // 按字节顺序计算校验和
    uint8_t bytes[] = {
        (uint8_t)ball_type,
        (uint8_t)(x >> 8), (uint8_t)x,
        (uint8_t)(y >> 8), (uint8_t)y,
        (uint8_t)(distance >> 8), (uint8_t)distance,
        (uint8_t)(angle >> 8), (uint8_t)angle
    };
    
    for(int i = 0; i < sizeof(bytes); i++) {
        calculated_checksum ^= bytes[i];
    }

    // 调试输出
    printf("计算校验和: 0x%02X, 接收校验和: 0x%02X\n", calculated_checksum, received_checksum);
    
    if (received_checksum == calculated_checksum) {
        USART_SendString("ACK\n");
    } else {
        USART_SendString("NAK\n");
    }
}


void OPENMV_USART_IRQHandler(void) {
    if (USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);

        if (ch == '$') {  // 检测到起始符
            rx_index = 0;
            rx_buf[rx_index++] = ch;
        } else if (rx_index > 0 && rx_index < RX_BUF_SIZE - 1) {
            rx_buf[rx_index++] = ch;
            if (ch == '\n') {  // 检测到结束符
                rx_buf[rx_index] = '\0';
                data_ready = 1;
                parse_received_data(rx_buf);
                rx_index = 0;
            }
        } else {
            rx_index = 0;  // 非法数据重置
        }
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}
