#include "uart.h"

uint8_t rx_buf[RX_BUF_SIZE];  // ������ջ�����
uint8_t rx_index = 0;         // ��ʼ������������
uint8_t data_ready = 0;       // ��ʼ������׼����־

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

void USART_SendString(const char* str)
{
    while (*str) {
        USART_Send((uint8_t)(*str));  // ����ÿ���ַ�
        str++;
    }
}

//У��Ͳ�ʹ��
void parse_received_data(uint8_t* data) {
    USART_SendString("Raw Data: ");
    USART_SendString((char*)data);
    USART_SendString("\n");

    uint8_t ball_type;
    int16_t x, y, distance, angle;
    uint8_t checksum;

    // ������ʽ�ַ���
    if (sscanf((char*)data, "$%hhu,%hd,%hd,%hd,%hd,%hhx",
               &ball_type, &x, &y, &distance, &angle, &checksum) != 6) {
        USART_SendString("ERR:FMT\n");
        return;
    }

    // У��ͼ���
    uint8_t calc = 0;
    uint8_t bytes[] = {
        (uint8_t)ball_type,
        (uint8_t)(x >> 8), (uint8_t)x,
        (uint8_t)(y >> 8), (uint8_t)y,
        (uint8_t)(distance >> 8), (uint8_t)distance,
        (uint8_t)(angle >> 8), (uint8_t)angle
    };
    
    for(int i=0; i<sizeof(bytes); i++) 
        calc ^= bytes[i];

    if(calc != checksum) {
        USART_SendString("CALC: ");
        USART_SendChecksum_Text(calc);
        USART_SendString(" NAK\n");
        return;
    }

    // ������Χ���
    if(x < 0 || x > 640 || y < 0 || y > 480 || distance < 0) {
        USART_SendString("ERR:RANGE\n");
        return;
    }

    // ��������
    ball_detected = ball_type;
    ball_x = x;
    ball_y = y;
    ball_distance = distance;
    ball_angle = angle;
    USART_SendString("ACK\n");
}

void OPENMV_USART_IRQHandler(void) {
    if(USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);
        
        // ֡��ʼ���
        if(ch == '$' && rx_index == 0) {
            rx_index = 0;
            rx_buf[rx_index++] = ch;
        } 
        // ���ݻ���
        else if(rx_index > 0 && rx_index < RX_BUF_SIZE-1) {
            rx_buf[rx_index++] = ch;
            if(ch == '\n') {  // ��⵽������
                rx_buf[rx_index] = '\0';
                
                // ͸����ʾԭʼ����
                USART_SendString("[STM32����] ");
                USART_SendString((char*)rx_buf);
                
                // ��������
                int16_t type, x, y, dist, angle;
                if(sscanf((char*)rx_buf, "$%hd,%hd,%hd,%hd,%hd", 
                         &type, &x, &y, &dist, &angle) == 5) {
                    // ���¿��Ʊ���
                    ball_detected = type;
                    ball_x = x;
                    ball_y = y;
                    ball_distance = dist;
                    ball_angle = angle;
                    
                    // �ش�ACK
                    USART_SendString("[STM32����] ACK\n");
                }
                rx_index = 0;
            }
        }
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}
