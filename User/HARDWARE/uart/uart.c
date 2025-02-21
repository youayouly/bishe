#include "uart.h"

// 接收缓冲区和状态变量
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;
uint8_t data_ready = 0;

#define RING_BUFFER_SIZE 2048  // 增大缓冲区至 2048 字节，容纳更多帧

typedef union {
    float f;
    uint8_t bytes[4];
} FloatUnion;

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

RingBuffer rxRingBuffer;

void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

int RingBuffer_Write(RingBuffer *rb, uint8_t data) {
    uint16_t next = (rb->head + 1) % RING_BUFFER_SIZE;
    if (next == rb->tail) {
        // 缓冲区满时暂停写入，返回错误
        return -1;
    }
    rb->buffer[rb->head] = data;
    rb->head = next;
    return 0;
}

int RingBuffer_Read(RingBuffer *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        return -1; // 缓冲区空
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return 0;
}

void UART_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    OPENMV_USART_GPIO_APBxClkCmd(OPENMV_USART_GPIO_CLK, ENABLE);
    OPENMV_USART_APBxClkCmd(OPENMV_USART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = OPENMV_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OPENMV_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = OPENMV_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(OPENMV_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = OPENMV_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 提高波特率至 230400
    USART_InitStructure.USART_BaudRate = 230400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(OPENMV_USARTx, &USART_InitStructure);
    USART_ITConfig(OPENMV_USARTx, USART_IT_RXNE, ENABLE);
    USART_Cmd(OPENMV_USARTx, ENABLE);

    RingBuffer_Init(&rxRingBuffer);
}

void USART_Send(uint8_t data) {
    while (USART_GetFlagStatus(OPENMV_USARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(OPENMV_USARTx, data);
}

void USART_SendString(const char* str) {
    while (*str) {
        USART_Send((uint8_t)(*str++));
    }
}

void USART_Send_Float(float value) {
    FloatUnion u;
    u.f = value;
    for (int i = 0; i < 4; i++) {
        USART_Send(u.bytes[i]);
    }
}

void USART_SendChecksum_Text(uint8_t checksum) {
    char buf[3];
    sprintf(buf, "%02X", checksum);
    USART_SendString(buf);
}

static int16_t prev_x = 0, prev_y = 0, prev_dist = 0, prev_angle = 0;
static uint8_t prev_valid = 0;

void parse_received_data(uint8_t* data) {
    uint8_t parsed_ball_detected;
    int16_t x, y, dist, angle;

    if (sscanf((char*)data, "$%hhu,%hd,%hd,%hd,%hd", 
               &parsed_ball_detected, &x, &y, &dist, &angle) != 5) {
        USART_SendString("ERR:FMT\n");
        return;
    }

    if (parsed_ball_detected == 0 && x == 0 && y == 0 && dist == 0 && angle == 0) {
        ball_detected = 0;
        USART_SendString("NO BALL\n");
        return;
    }

//    if ((parsed_ball_detected != 1 && parsed_ball_detected != 2) || (dist < 350 && dist > 0)) {
//        USART_SendString("ERR:VAL\n");
//        return;
//    }

    // 突变检测
    if (parsed_ball_detected == 1 || parsed_ball_detected == 2) {
        if (prev_valid) {
            float dynamic_threshold = prev_dist * 0.3f;
            dynamic_threshold = fmaxf(100.0f, fminf(300.0f, dynamic_threshold));
            if (abs(x - prev_x) > dynamic_threshold || 
                abs(y - prev_y) > dynamic_threshold || 
                abs(dist - prev_dist) > dynamic_threshold) {
                USART_SendString("ERR:JUMP\n");
                prev_x = x;
                prev_y = y;
                prev_dist = dist;
                prev_angle = angle;
                prev_valid = 1;
                return;
            }
        }
        prev_x = x;
        prev_y = y;
        prev_dist = dist;
        prev_angle = angle;
        prev_valid = 1;
    } else {
        prev_valid = 0;
    }

    if (angle > 32767) {
        angle -= 65536;
    }

    ball_detected = parsed_ball_detected;
    ball_x = x;
    ball_y = y;
    ball_distance = dist;
    ball_angle = angle;

    // 仅每 5 次解析发送一次响应（减少发送开销）
    static uint8_t response_counter = 0;
    response_counter = (response_counter + 1) % 5;
    if (response_counter == 0) {
        char response[32];
        int len = snprintf(response, sizeof(response), "$%d,%d,%d,%d,%d\n",
                           ball_detected, ball_x, ball_y, ball_distance, ball_angle);
        if (len > 0) {
            USART_SendString(response);
        } else {
            USART_SendString("ERR:RESP\n");
        }
    }
}

void ProcessReceivedData(void) {
    static uint8_t rx_packet[RX_BUF_SIZE];
    static uint16_t rx_packet_idx = 0;
    uint8_t ch;

    // 批量处理缓冲区数据
    while (RingBuffer_Read(&rxRingBuffer, &ch) == 0) {
        if (ch == '$') {
            rx_packet_idx = 0;
        }
        
        if (rx_packet_idx < RX_BUF_SIZE - 1) {
            rx_packet[rx_packet_idx++] = ch;
            if (ch == '\n') {
                rx_packet[rx_packet_idx] = '\0';
                parse_received_data(rx_packet);
                rx_packet_idx = 0;
            }
        } else {
            rx_packet_idx = 0; // 溢出重置
        }
    }
}

void OPENMV_USART_IRQHandler(void) {
    if (USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);
        if (RingBuffer_Write(&rxRingBuffer, ch) != 0) {
            // 缓冲区满，丢弃新数据（避免阻塞）
        }
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}

#define MEDIAN_FILTER_SIZE 7
float median_filter(float new_val) {
    static float buffer[MEDIAN_FILTER_SIZE] = {0};
    static uint8_t index = 0;
    
    buffer[index] = new_val;
    index = (index + 1) % MEDIAN_FILTER_SIZE;
    
    float sorted[MEDIAN_FILTER_SIZE];
    memcpy(sorted, buffer, sizeof(sorted));
    for (int i = 0; i < MEDIAN_FILTER_SIZE - 1; i++) {
        for (int j = i + 1; j < MEDIAN_FILTER_SIZE; j++) {
            if (sorted[i] > sorted[j]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }
    return sorted[MEDIAN_FILTER_SIZE / 2];
}

#define SPIKE_WINDOW 15
float dynamic_spike_filter(float new_val) {
    static float history[SPIKE_WINDOW] = {0};
    static uint8_t init_count = 0;
    static uint8_t idx = 0;
    static float sum = 0, sq_sum = 0;
    
    if (init_count < SPIKE_WINDOW) {
        history[init_count++] = new_val;
        sum += new_val;
        sq_sum += new_val * new_val;
        return new_val;
    }
    
    sum -= history[idx];
    sq_sum -= history[idx] * history[idx];
    history[idx] = new_val;
    sum += new_val;
    sq_sum += new_val * new_val;
    idx = (idx + 1) % SPIKE_WINDOW;
    
    float mean = sum / SPIKE_WINDOW;
    float variance = (sq_sum - sum * mean) / SPIKE_WINDOW;
    float std_dev = sqrtf(variance);
    float threshold = 3 * std_dev;
    
    return (fabs(new_val - mean) > threshold) ? mean : new_val;
}