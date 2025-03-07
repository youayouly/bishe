#include "Header.h"

volatile u8 delay_50,delay_flag; 		//延时变量			
u16 Voltage;							//电压变量，放大100倍储存
u8 Flag_Stop;							//电机启停标志位
u8 PS2_ON_Flag = 0,APP_ON_Flag = 0,ROS_ON_Flag = 0;		//默认所有方式不控制
u8 Car_Num;								//车型号码选择
u8 Flag_Show = 1;						//显示标志位，默认开启，长按切换到上位机模式，此时关闭
float Perimeter; 						//轮子的周长
float Wheelspacing; 					//轮子的轮距
u16 DISTANCE=0,ANGLE=0;
u8 one_lap_data_success_flag=0;         //雷达数据完成一圈的接收标志位
int lap_count=0;//当前雷达这一圈数据有多少个点
int PointDataProcess_count=0,test_once_flag=0,Dividing_point=0;//雷达接收数据点的计算值、接收到一圈数据最后一帧数据的标志位、需要切割数据的数据数
/**************************************************************************  
Function: Main function
Input   : none
Output  : none
函数功能：主函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
int main(void)
{	
	u32 Voltage_Sum = 0;
	u8 Voltage_Count = 0;				//电压计算相关变量
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级分组
	LED_GPIO_Config();					//LED初始化，PC2和PC3
	Key_GPIO_Config();					//按键初始化，PA0和PC13  按键来使用雷达
	BEEP_GPIO_Config();					//蜂鸣器初始化，PA15
	OLED_Init();						//OLED初始化
	Encoder_Init();						//编码器初始化，TIM4和TIM8 
	Motor_Init(7199,0);					//电机PWM初始化，TIM3
	DEBUG_USART_Init();					//调试串口设置，串口1，波特率115200
	BLUETOOTH_USART_Init();				//蓝牙串口设置，串口3，波特率9600
	LIDAR_USART_Init();					//雷达串口设置，串口5，波特率115200
  
  
  //串口2
  UART_Init();
//  USART_Send('1');
  
  
	PS2_Init();							//初始化PS2手柄接口
	PS2_SetInit();						//PS2手柄配置为模拟量模式
	Car_Select_ADC_Init();				//车型选择ADC初始化
	Voltage_ADC_Init();					//用于测量电压
//	Distance_Cap_Init(0XFFFF,72-1);		//用于测量超声波的距离，默认注释，具体看capture.h
//	PWM_Cap_Init(0XFFFF,72-1);			//用于4路航模遥控初始化，默认注释，具体看capture.h   /*超声波和航模只能使用其一*/
	MPU6050_Init();										//MPU6050初始化
	Robot_Select();
	BEEP_ON;													//按键按下蜂鸣器提示
	delay_ms(200);
	BEEP_OFF;
    if(Car_Num == Akm_Car)
		Servo_Init(9999,71);									//阿克曼车型初始化舵机
    Servo_Init3(9999,71);
    
	Car_Perimeter_Init();										//初始化轮子周长和轮距
	TIMING_TIM_Init(7199,49);									//5ms中断控制，大部分控制逻辑在里面 ->监控按键，control 雷达-》总控制 其他是5ms更新一次数据
	
//   while (1) {
//      if (data_ready) {  // 如果数据已接收
//          data_ready = 0;  // 重置标志位
//          USART_SendString((const char*)rx_buf);  // 发送接收到的数据
//          USART_Send('\n');  // 添加换行符
//      }
//    }
    uint8_t processing_buf[RX_BUF_SIZE];
    
   while(1)
	{
//      if (data_ready) {
//            memcpy(processing_buf, rx_buf, RX_BUF_SIZE);
//            data_ready = 0;
//            parse_received_data(processing_buf);
//        }
//      USART_SendString((const char*)rx_buf);  // 发送接收到的数据
//    if (data_ready) {  // 如果数据已接收
//          data_ready = 0;  // 重置标志位
//          USART_SendString((const char*)rx_buf);  // 发送接收到的数据
//          USART_Send('\n');  // 添加换行符
//      }
//    
      ProcessReceivedData();
    
		Robot_Select();
		if(Flag_Show)											//正常显示，非上位机模式
		{
			LED2_OFF;
			PS2_Read();											//手柄数据读取
			Show();												//显示屏
           APP_Show();											//手机蓝牙显示
			
			Voltage_Sum += Get_Voltage();						//电压测量，每50次取一次平均
      
			if((++Voltage_Count) == 5)
				Voltage = Voltage_Sum/5,Voltage_Count = 0,Voltage_Sum = 0;
		}
		else													//进入上位机模式
		{
			LED2_ON;											//LED2常亮指示进入了上位机模式
			DataScope();										//上位机示波器显示
		}
		delay_flag=1;	//使用50ms延时时，雷达数据会出现异常
		delay_50=0;
		while(delay_flag);	     								//通过定时中断实现的50ms延时，主要用于示波器	
      //避免屏幕刷新的过快  主要用于示波器	一般是20次每秒
	}
}


/***********************************END OF FILE********************************/

