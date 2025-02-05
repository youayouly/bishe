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
    // 增加长度校验
    if(strlen((char*)data) < 20) {
        USART_SendString("ERR:SHORT\n");
        return;
    }
    
    // 使用安全解析方式
    int parsed = sscanf((char*)data, "$%hu,%hd,%hd,%hd,%hd,%hhx", 
                       &ball_detected, &ball_x, &ball_y, &ball_distance, &ball_angle, &checksum);
                       
    if(parsed != 6) {
        USART_SendString("ERR:FMT\n");
        return;
    }
    
    // 校验和验证（优化计算顺序）
    uint8_t calc_checksum = 0;
    uint8_t* ptr = (uint8_t*)&ball_detected;
    for(int i=0; i<2; i++) calc_checksum ^= ptr[i]; // ball_type是uint8_t
    
    ptr = (uint8_t*)&ball_x;
    for(int i=0; i<2; i++) calc_checksum ^= ptr[i];
    
    ptr = (uint8_t*)&ball_y;
    for(int i=0; i<2; i++) calc_checksum ^= ptr[i];
    
    ptr = (uint8_t*)&ball_distance;
    for(int i=0; i<2; i++) calc_checksum ^= ptr[i];
    
    ptr = (uint8_t*)&ball_angle;
    for(int i=0; i<2; i++) calc_checksum ^= ptr[i];
    
    if(calc_checksum != checksum) {
        USART_SendString("NAK\n");
        return;
    }
    
    // 数据有效性检查
    if(abs(ball_x) > 640 || abs(ball_y) > 480 || ball_distance < 0) {
        USART_SendString("ERR:INVALID\n");
        return;
    }
    
    // 更新球数据
    ball_last_tick = GET_TICK();
    ball_detected = 1;
    USART_SendString("ACK\n");
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
