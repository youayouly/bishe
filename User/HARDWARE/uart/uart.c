#include "uart.h"

uint8_t rx_buf[RX_BUF_SIZE];  // 定义接收缓冲区
uint8_t rx_index = 0;         // 初始化缓冲区索引
uint8_t data_ready = 0;       // 初始化数据准备标志

#define RING_BUFFER_SIZE 256  // 根据需要调整缓冲区大小

typedef union {
    float f;
    uint8_t bytes[4];
} FloatUnion;

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

int RingBuffer_Write(RingBuffer *rb, uint8_t data) {
    uint16_t next = (rb->head + 1) % RING_BUFFER_SIZE;
    if (next == rb->tail) {
        // 缓冲区已满，无法写入
        return -1;
    }
    rb->buffer[rb->head] = data;
    rb->head = next;
    return 0;
}

int RingBuffer_Read(RingBuffer *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        // 缓冲区为空，无法读取
        return -1;
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return 0;
}
RingBuffer rxRingBuffer;




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
    
        // 初始化环形缓冲区
    RingBuffer_Init(&rxRingBuffer);
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


void USART_Send_Float(float value) {
    FloatUnion u;
    u.f = value;
    for (int i = 0; i < 4; i++) {
        USART_Send(u.bytes[i]);
    }
}

void USART_SendString(const char* str)
{
    while (*str) {
        USART_Send((uint8_t)(*str));  // 发送每个字符
        str++;
    }
}

static uint16_t ack_counter = 0;  // ACK 发送计数器
void parse_received_data(uint8_t* data) {
  
    uint8_t parsed_ball_detected;
    int16_t x, y, dist, angle;
  
  //检验distance error正负值
//    float ball_distance1=ball_distance /1000.0f;
//    float error_distance = ball_distance1 - TARGET_DISTANCE; //0.9 0.35
  
  //这个是更新函数
    if (sscanf((char*)data, "$%hhd,%hd,%hd,%hd,%hd", 
               &parsed_ball_detected, &x, &y, &dist, &angle) != 5) {
        USART_SendString("ERR:FMT\n");
        return;
    }

//    // 合法性检查
//    if (x < 0 || x > 640 || y < 0 || y > 480 || dist < 0) {
//        USART_SendString("ERR:RANGE\n");
//        return;
//    }

    // 如果所有数据均为0，则不处理
    if (parsed_ball_detected == 0 && x == 0 && y == 0 && dist == 0 && angle == 0) {
           // 也可以发送一个 "NO BALL\n" 之类的提示
          USART_SendString("NO BALL\n");
          return;
    }

    // 更新全局变量
    ball_detected = parsed_ball_detected;
    ball_x = x;
    ball_y = y;
    ball_distance = dist;
    ball_angle = angle;
    // 发送 ACK 回复
    //USART_SendString("ACK\n");
//    char error_distance_str[20];
//    snprintf(error_distance_str, sizeof(error_distance_str), "%.2f", error_distance);

    // 发送更新后的数据
    char response[100];
    int len = snprintf(response, sizeof(response), "$%hhd,%hd,%hd,%hd,%hd\n",
                       ball_detected, ball_x, ball_y, ball_distance, ball_angle);
    if (len > 0) {
        USART_SendString(response);  // 发送返回的数据
      
//        USART_SendString(error_distance_str);
    } else {
        USART_SendString("ERR:RESP\n");  // 发送错误信息
    }
}

void ProcessReceivedData(void) {
    static uint8_t rx_buf[RX_BUF_SIZE];
    static uint16_t rx_index = 0;
    uint8_t ch;

    while (RingBuffer_Read(&rxRingBuffer, &ch) == 0) {
        if (ch == '$') {
            rx_index = 0;
        }

        if (rx_index < RX_BUF_SIZE - 1) {
            rx_buf[rx_index++] = ch;
            if (ch == '\n') {
                rx_buf[rx_index] = '\0';
                
                parse_received_data(rx_buf);
                rx_index = 0;
            }
        } else {
            // 缓冲区溢出，重置索引
            rx_index = 0;
        }
    }
}

// UART中断处理函数
void OPENMV_USART_IRQHandler(void) {
    if (USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);
        RingBuffer_Write(&rxRingBuffer, ch);
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}

