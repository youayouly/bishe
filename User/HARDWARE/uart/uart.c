#include "uart.h"


#define RX_BUF_SIZE 64
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t data_ready = 0;


// 串口4 uart
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

	//配置USART为中断源 
	NVIC_InitStructure.NVIC_IRQChannel = OPENMV_USART_IRQ;
	//抢断优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//使能中断 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);


	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = OPENMV_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
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


// 串口中断服务函数
void USART_receive(void) 
{
    //USART_IT_RXNE是中断标志，指示 USART 接收寄存器有数据可读取
    if(USART_GetITStatus(OPENMV_USARTx, USART_IT_RXNE) != RESET) {
      
        //存储相关的数据
        uint8_t ch = USART_ReceiveData(OPENMV_USARTx);
        
        // 检测帧结束符
        if(ch == '\n') {
            rx_buf[rx_index] = '\0';  // 字符串终止符
            data_ready = 1;
            rx_index = 0;
        } else {
            // 缓存未满时存储数据
            if(rx_index < RX_BUF_SIZE-1) {
                rx_buf[rx_index++] = ch;
            }
        }
        USART_ClearITPendingBit(OPENMV_USARTx, USART_IT_RXNE);
    }
}