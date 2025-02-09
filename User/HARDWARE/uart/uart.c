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
  
  //����distance error����ֵ
//    float ball_distance1=ball_distance /1000.0f;
//    float error_distance = ball_distance1 - TARGET_DISTANCE; //0.9 0.35
  
  //����Ǹ��º���
    if (sscanf((char*)data, "$%hhd,%hd,%hd,%hd,%hd", 
               &parsed_ball_detected, &x, &y, &dist, &angle) != 5) {
        USART_SendString("ERR:FMT\n");
        return;
    }

//    // �Ϸ��Լ��
//    if (x < 0 || x > 640 || y < 0 || y > 480 || dist < 0) {
//        USART_SendString("ERR:RANGE\n");
//        return;
//    }

    // ����������ݾ�Ϊ0���򲻴���
    if (parsed_ball_detected == 0 && x == 0 && y == 0 && dist == 0 && angle == 0) {
           // Ҳ���Է���һ�� "NO BALL\n" ֮�����ʾ
          USART_SendString("NO BALL\n");
          return;
    }

    // ����ȫ�ֱ���
    ball_detected = parsed_ball_detected;
    ball_x = x;
    ball_y = y;
    ball_distance = dist;
    ball_angle = angle;
    // ���� ACK �ظ�
    //USART_SendString("ACK\n");
//    char error_distance_str[20];
//    snprintf(error_distance_str, sizeof(error_distance_str), "%.2f", error_distance);

    // ���͸��º������
    char response[100];
    int len = snprintf(response, sizeof(response), "$%hhd,%hd,%hd,%hd,%hd\n",
                       ball_detected, ball_x, ball_y, ball_distance, ball_angle);
    if (len > 0) {
        USART_SendString(response);  // ���ͷ��ص�����
      
//        USART_SendString(error_distance_str);
    } else {
        USART_SendString("ERR:RESP\n");  // ���ʹ�����Ϣ
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

