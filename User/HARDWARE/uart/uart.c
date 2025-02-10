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

    // 解析数据
    if (sscanf((char*)data, "$%hhu,%hd,%hd,%hd,%hd", 
               &parsed_ball_detected, &x, &y, &dist, &angle) != 5) {
        USART_SendString("ERR:FMT\n");
        return;
    }

    // 如果所有数据均为0，则不处理
    if (parsed_ball_detected == 0 && x == 0 && y == 0 && dist == 0 && angle == 0) {
        USART_SendString("NO BALL\n");
        return;
    }

    // 定义差分阈值
    const int16_t DIST_DIFF_THRESHOLD = 300; // 根据实际情况设置阈值
    const int16_t ANGLE_DIFF_THRESHOLD = 300; // 根据实际情况设置阈值

    // 检查距离和角度的差分是否超过阈值
          
      // 如果上一次数据为0（无有效检测），则直接更新，不做差分判断
//      if (ball_distance == 0 && ball_angle == 0) {
//          ball_distance = dist;
//          ball_angle = angle;
//      } else if (abs(dist - ball_distance) > DIST_DIFF_THRESHOLD ||
//                 abs(angle - ball_angle) > ANGLE_DIFF_THRESHOLD) 
//      {
//          USART_SendString("ERR:SPIKE\n");
//          return; // 忽略异常数据
//      }


        // 多级滤波
    //float temp_dist = median_filter(dist);          // 中值滤波
    //temp_dist=Mean_Filter_Left(temp_dist);          //滑动滤波
//    temp_dist = dynamic_spike_filter(temp_dist);    // 动态阈值滤波
    
//    float temp_angle = median_filter(angle);  // 对距离进行中值滤波
//    temp_dist = dynamic_spike_filter(temp_angle);  // 对中值滤波后的结果进行动态阈值检测

    // 更新全局变量
    ball_detected = parsed_ball_detected;
    ball_x = x;
    ball_y = y;
    ball_distance = dist;
    ball_angle = angle;

    // 发送更新后的数据
    char response[100];
    int len = snprintf(response, sizeof(response), "$%hhd,%hd,%hd,%hd,%hd\n",
                       ball_detected, ball_x, ball_y, ball_distance, ball_angle);
    if (len > 0) {
        USART_SendString(response);
    } else {
        USART_SendString("ERR:RESP\n");
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

// 中值滤波实现
#define MEDIAN_FILTER_SIZE 5

// 中值滤波函数，返回滤波后的值
float median_filter(float new_val) {
    static float buffer[MEDIAN_FILTER_SIZE] = {0};  // 存储最近的几个值
    static uint8_t index = 0;  // 缓冲区的索引
    
    // 更新缓冲区，循环覆盖
    buffer[index] = new_val;
    index = (index + 1) % MEDIAN_FILTER_SIZE;
    
    // 对缓冲区的值进行排序，找到中值
    float sorted[MEDIAN_FILTER_SIZE];
    memcpy(sorted, buffer, sizeof(sorted));
    for(int i = 0; i < MEDIAN_FILTER_SIZE - 1; i++) {
        for(int j = i + 1; j < MEDIAN_FILTER_SIZE; j++) {
            if(sorted[i] > sorted[j]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }
    
    // 返回中值
    return sorted[MEDIAN_FILTER_SIZE / 2];
}


#define SPIKE_WINDOW 10  // 动态阈值窗口大小

// 动态阈值检测函数
float dynamic_spike_filter(float new_val) {
    static float history[SPIKE_WINDOW] = {0};  // 存储最近的几个值
    static uint8_t idx = 0;  // 当前值的索引
    static float sum = 0, sq_sum = 0;  // 求和与平方和
    
    // 移除历史数据中的最旧值
    sum -= history[idx];
    sq_sum -= history[idx] * history[idx];
    
    // 添加新的值到历史数据
    history[idx] = new_val;
    sum += new_val;
    sq_sum += new_val * new_val;
    
    // 更新索引，保持窗口大小
    idx = (idx + 1) % SPIKE_WINDOW;
    
    // 计算当前数据的均值和标准差
    float mean = sum / SPIKE_WINDOW;
    float variance = (sq_sum - sum * mean) / SPIKE_WINDOW;
    float std_dev = sqrtf(variance);  // 标准差
    
    // 计算动态阈值，使用3倍标准差（3σ原则）
    float threshold = 3 * std_dev;
    
    // 如果当前值与均值的差距超过阈值，则认为是异常值
    return (fabs(new_val - mean) > threshold) ? mean : new_val;
}



