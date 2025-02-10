#include "uart.h"

uint8_t rx_buf[RX_BUF_SIZE];  // ������ջ�����
uint8_t rx_index = 0;         // ��ʼ������������
uint8_t data_ready = 0;       // ��ʼ������׼����־

#define RING_BUFFER_SIZE 256  // ������Ҫ������������С

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
        // �������������޷�д��
        return -1;
    }
    rb->buffer[rb->head] = data;
    rb->head = next;
    return 0;
}

int RingBuffer_Read(RingBuffer *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        // ������Ϊ�գ��޷���ȡ
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

    // ����USARTΪ�ж�Դ
    NVIC_InitStructure.NVIC_IRQChannel = OPENMV_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // �������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // ʹ���ж�
    NVIC_Init(&NVIC_InitStructure);

    // ���ô��ڵĹ�������
    USART_InitStructure.USART_BaudRate = OPENMV_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // ��ɴ��ڵĳ�ʼ������
    USART_Init(OPENMV_USARTx, &USART_InitStructure);

    // ʹ�ܴ��ڽ����ж�
    USART_ITConfig(OPENMV_USARTx, USART_IT_RXNE, ENABLE);  

    // ʹ�ܴ���
    USART_Cmd(OPENMV_USARTx, ENABLE);    
    
        // ��ʼ�����λ�����
    RingBuffer_Init(&rxRingBuffer);
}

// �޸�Ϊ���� 8 λУ��͵��ı���ת��Ϊ 2 λʮ�������ַ�����
void USART_SendChecksum_Text(uint8_t checksum) {
    char buf[3];  // ����ת������ַ�������ʽ"XX" + '\0'
    sprintf(buf, "%02X", checksum); // ת��Ϊ2λʮ�������ַ���������"89"
    USART_SendString(buf);          // ��������ַ���
}

void USART_Send(uint8_t data)
{
    // �ȴ����ͻ�����Ϊ��
    while (USART_GetFlagStatus(OPENMV_USARTx, USART_FLAG_TXE) == RESET);
    // ��������
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
        USART_Send((uint8_t)(*str));  // ����ÿ���ַ�
        str++;
    }
}

static uint16_t ack_counter = 0;  // ACK ���ͼ�����
void parse_received_data(uint8_t* data) {
    uint8_t parsed_ball_detected;
    int16_t x, y, dist, angle;

    // ��������
    if (sscanf((char*)data, "$%hhu,%hd,%hd,%hd,%hd", 
               &parsed_ball_detected, &x, &y, &dist, &angle) != 5) {
        USART_SendString("ERR:FMT\n");
        return;
    }

    // ����������ݾ�Ϊ0���򲻴���
    if (parsed_ball_detected == 0 && x == 0 && y == 0 && dist == 0 && angle == 0) {
        USART_SendString("NO BALL\n");
        return;
    }

    // ��������ֵ
    const int16_t DIST_DIFF_THRESHOLD = 300; // ����ʵ�����������ֵ
    const int16_t ANGLE_DIFF_THRESHOLD = 300; // ����ʵ�����������ֵ

    // ������ͽǶȵĲ���Ƿ񳬹���ֵ
          
      // �����һ������Ϊ0������Ч��⣩����ֱ�Ӹ��£���������ж�
//      if (ball_distance == 0 && ball_angle == 0) {
//          ball_distance = dist;
//          ball_angle = angle;
//      } else if (abs(dist - ball_distance) > DIST_DIFF_THRESHOLD ||
//                 abs(angle - ball_angle) > ANGLE_DIFF_THRESHOLD) 
//      {
//          USART_SendString("ERR:SPIKE\n");
//          return; // �����쳣����
//      }


        // �༶�˲�
    //float temp_dist = median_filter(dist);          // ��ֵ�˲�
    //temp_dist=Mean_Filter_Left(temp_dist);          //�����˲�
//    temp_dist = dynamic_spike_filter(temp_dist);    // ��̬��ֵ�˲�
    
//    float temp_angle = median_filter(angle);  // �Ծ��������ֵ�˲�
//    temp_dist = dynamic_spike_filter(temp_angle);  // ����ֵ�˲���Ľ�����ж�̬��ֵ���

    // ����ȫ�ֱ���
    ball_detected = parsed_ball_detected;
    ball_x = x;
    ball_y = y;
    ball_distance = dist;
    ball_angle = angle;

    // ���͸��º������
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
            // �������������������
            rx_index = 0;
        }
    }
}

// UART�жϴ�����
void OPENMV_USART_IRQHandler(void) {
    if (USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);
        RingBuffer_Write(&rxRingBuffer, ch);
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}

// ��ֵ�˲�ʵ��
#define MEDIAN_FILTER_SIZE 5

// ��ֵ�˲������������˲����ֵ
float median_filter(float new_val) {
    static float buffer[MEDIAN_FILTER_SIZE] = {0};  // �洢����ļ���ֵ
    static uint8_t index = 0;  // ������������
    
    // ���»�������ѭ������
    buffer[index] = new_val;
    index = (index + 1) % MEDIAN_FILTER_SIZE;
    
    // �Ի�������ֵ���������ҵ���ֵ
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
    
    // ������ֵ
    return sorted[MEDIAN_FILTER_SIZE / 2];
}


#define SPIKE_WINDOW 10  // ��̬��ֵ���ڴ�С

// ��̬��ֵ��⺯��
float dynamic_spike_filter(float new_val) {
    static float history[SPIKE_WINDOW] = {0};  // �洢����ļ���ֵ
    static uint8_t idx = 0;  // ��ǰֵ������
    static float sum = 0, sq_sum = 0;  // �����ƽ����
    
    // �Ƴ���ʷ�����е����ֵ
    sum -= history[idx];
    sq_sum -= history[idx] * history[idx];
    
    // ����µ�ֵ����ʷ����
    history[idx] = new_val;
    sum += new_val;
    sq_sum += new_val * new_val;
    
    // �������������ִ��ڴ�С
    idx = (idx + 1) % SPIKE_WINDOW;
    
    // ���㵱ǰ���ݵľ�ֵ�ͱ�׼��
    float mean = sum / SPIKE_WINDOW;
    float variance = (sq_sum - sum * mean) / SPIKE_WINDOW;
    float std_dev = sqrtf(variance);  // ��׼��
    
    // ���㶯̬��ֵ��ʹ��3����׼�3��ԭ��
    float threshold = 3 * std_dev;
    
    // �����ǰֵ���ֵ�Ĳ�೬����ֵ������Ϊ���쳣ֵ
    return (fabs(new_val - mean) > threshold) ? mean : new_val;
}



