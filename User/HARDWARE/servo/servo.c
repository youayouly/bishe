#include "servo.h"   

int SERVO3_INIT=1800;
  /**************************************************************************
Function: Servo_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
函数功能：舵机pwm输出初始化
入口参数: 预装载值和预分频器 
返回  值：无
**************************************************************************/	 	
void Servo_Init3(u16 arr,u16 psc)
{

	    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 使能GPIO时钟（PA3）
    SERVO3_TIM_APBxClock_FUN(SERVO3_GPIO_CLK, ENABLE);

    // 配置PA3为复用推挽输出（用于PWM信号）
    GPIO_InitStruct.GPIO_Pin = SERVO3_GPIO_PIN;  // PA3
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // 输出速度50MHz
    GPIO_Init(SERVO3_GPIO_PORT, &GPIO_InitStruct);

    // 使能TIM2时钟
    SERVO3_TIM_APBxClock_FUN(SERVO3_TIM_CLK, ENABLE);

    // 配置定时器TIM2
    TIM_TimeBaseInitStruct.TIM_Period = arr;  // 自动重装值
    TIM_TimeBaseInitStruct.TIM_Prescaler = psc;  // 预分频器
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数模式
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频
    TIM_TimeBaseInit(SERVO3_TIM, &TIM_TimeBaseInitStruct);

    // 配置PWM模式（TIM2通道4）
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;  // PWM模式1
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  // 使能输出
    TIM_OCInitStruct.TIM_Pulse = 0;  // 初始占空比
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;  // 高电平有效
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;  // 空闲状态为复位
    SERVO3_TIM_OCXInit_FUN(SERVO3_TIM, &TIM_OCInitStruct);  // 初始化TIM2通道4

    // 使能TIM2通道4的预装载寄存器
    SERVO3_TIM_OCXPreloadConfig_FUN(SERVO3_TIM, TIM_OCPreload_Enable);

    // 使能TIM2的ARR预装载寄存器
    TIM_ARRPreloadConfig(SERVO3_TIM, ENABLE);

    // 使能TIM2
    TIM_Cmd(SERVO3_TIM, ENABLE);

    // 设置初始占空比
    SERVO3_TIM_SetCompareX_FUN(SERVO3_TIM, SERVO3_INIT);  // 设置TIM2通道4的捕获比较寄存器值
}

void Set_Servo_PWM(uint16_t pwm_value)
{
    // 直接调用在 Servo_Init3 中用到的宏，
    // 用于更新定时器的捕获比较寄存器，从而调整 PWM 占空比
    SERVO3_TIM_SetCompareX_FUN(SERVO3_TIM, pwm_value);
}
