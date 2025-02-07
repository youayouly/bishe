
#include "control.h"
#include "Lidar.h"
#include "MPU6050.h"

float Move_X = 0,Move_Z = 0;						//目标速度和目标转向速度
float PWM_Left,PWM_Right;					//左右电机PWM值
float RC_Velocity,RC_Turn_Velocity;			//遥控控制的速度
u8 Mode = 0;								//模式选择，默认是普通的控制模式
Motor_parameter MotorA,MotorB;				//左右电机相关变量
int Servo_PWM = SERVO_INIT;					//阿克曼舵机相关变量
u8 Lidar_Detect = Lidar_Detect_ON;			//电磁巡线模式雷达检测障碍物，默认开启
float CCD_Move_X = 0.3;						//CCD巡线速度
float ELE_Move_X = 0.3;						//电磁巡线速度
u8 Ros_count=0;
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据   
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;



int16_t ball_x = 0, ball_y = 0;
uint8_t ball_detected = 0;
uint32_t ball_last_tick=0;
uint32_t sys_tick = 0;
uint8_t checksum = 0;

int16_t ball_angle = 0;
int16_t ball_distance = 0;

float last_error = 0.0f; // 需改为全局变量
uint8_t stable_count = 0;
float filtered_x = BALL_CENTER_X;

float Vx = 0.0f, Vz = 0.0f;
float distance_ratio = 0.0f;

//停止函数
// 增强版停止控制
void Stop_Motor_With_Kinematics(void) 
{
    // 运动学清零
    Get_Target_Encoder(0.0f, 0.0f);
    Move_X = 0; 
    Move_Z = 0;
    
    // 电机控制清零
    MotorA.Target_Encoder = MotorA.Current_Encoder; // 保持当前编码器值
    MotorB.Target_Encoder = MotorB.Current_Encoder;
    
    // 强制PWM输出0
    Set_Pwm(0, 0);
    
    // 重置所有控制器状态
    Reset_Incremental_PI_Left();
    Reset_Incremental_PI_Right();
    MotorA.Current_Encoder = MotorB.Current_Encoder = 0;
    
    // 清空跟踪状态

    last_error = 0.0f;
}
BallFilter ball_filter = {0};

// 数据有效性检查
uint8_t is_valid_data(int16_t x, int16_t dist) {
    return (x >= 0 && x < 640 && 
           dist > 10 && dist < 800 && // 10cm~8m
           (sys_tick - ball_filter.last_update) < 200); // 1秒超时
}

// 低通滤波
void update_ball_filter(int16_t x, int16_t dist) {
    const float alpha = 0.3f;
    if(ball_filter.last_update == 0) {
        ball_filter.x = x;
        ball_filter.dist = dist;
    } else {
        ball_filter.x = alpha*x + (1-alpha)*ball_filter.x;
        ball_filter.dist = alpha*dist + (1-alpha)*ball_filter.dist;
    }
    ball_filter.last_update = sys_tick;
}

// 定义全局变量（实际内存分配）
OperationMode current_mode2 = LIDAR_AVOID; // 初始化为避障模式


/**************************************************************************
Function: Control Function
Input   : none
Output  : none
函数功能：5ms定时中断控制函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
int TIMING_TIM_IRQHandler(void)
{
  
  static uint8_t stop_cnt = 0;
	static u8 Count_CCD = 0;								//调节CCD控制频率
	if(TIM_GetITStatus( TIMING_TIM, TIM_IT_Update) != RESET ) //不等于1 ==0
	{			
		TIM_ClearITPendingBit(TIMING_TIM , TIM_IT_Update);
    
    sys_tick++; // 每5ms自增1
        // ...原有逻辑...
    
		Get_Velocity_From_Encoder();								//读取左右编码器的值且转换成速度
        Get_KeyVal();		 		
		if(delay_flag)												//50ms延时
		{
			if(++delay_50==10) delay_50=0,delay_flag=0;            	//给主函数提供50ms的精准延时，主要是用于上位机示波器
			else if(delay_50>10)	delay_50=0,delay_flag=0;
		}
    switch(Mode) {
            case Normal_Mode:
                if(++Ros_count == 10) {
                    Get_Angle(2);
                    Ros_count=0;
                }
                break;
                
            case Lidar_Avoid_Mode:
                switch(current_mode2) {
                    case LIDAR_AVOID:
                        if(ball_detected) {
                            current_mode2 = BALL_TRACKING;
                            Stop_Motor_With_Kinematics();
                        } else {
                            Lidar_Avoid();  // 保持原有避障逻辑
                            Get_Target_Encoder(Move_X, Move_Z); // 新增运动学转换
                        }
                        break;
                        
                    case BALL_TRACKING:
                        
                        if(!ball_detected) {
                                current_mode2 = LIDAR_AVOID;
                                Stop_Motor_With_Kinematics();
                            } else {
                                Track_Ball();
                                
                                // 新增停止条件（速度+位置）
                                if(fabsf(MotorA.Current_Encoder) < 0.05f && 
                                   fabsf(MotorB.Current_Encoder) < 0.05f &&
                                   fabsf(ball_filter.x - BALL_CENTER_X) < 30 &&
                                   ball_filter.dist < SAFE_DISTANCE*1.1f) 
                                {
                                   
                                    if(++stop_cnt > 20) { // 100ms稳定
                                        Stop_Motor_With_Kinematics();
                                        current_mode2 = LIDAR_AVOID;
                                        stop_cnt = 0;
                                    }
                                } else {
                                    stop_cnt = 0;
                                }
                            }
                            break;
                }
                break;
                
            case Lidar_Follow_Mode:
                Lidar_Follow();
                break;
                
            case Lidar_Along_Mode:
                Lidar_along_wall();
                break;
        }
    
    
//		else if(Mode == ELE_Line_Patrol_Mode)						//电磁巡线模式
//			ELE_Mode();
//		else														//CCD模式
//		{
//			if(++Count_CCD == 4)									//调节控制频率，4*5 = 20ms控制一次
//			{
//				Count_CCD = 0;
//				CCD_Mode();											
//			}
//			else if(Count_CCD>4)
//				Count_CCD = 0;
//		}			
		
//		else 
//			Ultrasonic_Follow();									//超声波跟随
    
//		Get_Target_Encoder(Move_X,Move_Z);							//运动学逆解解，转换成编码器的目标速度
    
    //中间如果出现了问题
		if(Turn_Off()==Normal)										//检查电机是否关闭，电压是否不足
		{				
			Get_Motor_PWM();										//转换成驱动电机的pwm
		}
		else
		{
			MotorA.Motor_Pwm  = 0,MotorB.Motor_Pwm = 0,Servo_PWM = SERVO_INIT;
		}
		Set_Pwm(-MotorA.Motor_Pwm,MotorB.Motor_Pwm);				//驱动电机
	}		 	
	return 0;
}



void Track_Ball(void) {
  
    static float last_error = 0;
    static uint8_t align_cnt = 0;
    
    // 数据有效性检查
    if(!is_valid_data(ball_x, ball_distance)) {
        Stop_Motor_With_Kinematics();
        return;
    }
    
    // 更新滤波数据
    update_ball_filter(ball_x, ball_distance);
    float cur_x = ball_filter.x;
    float cur_dist = ball_filter.dist;
    
    // 计算误差
    float error_x = BALL_CENTER_X - cur_x;
    float error_d = error_x - last_error;
    last_error = error_x;
    
    // 距离控制策略
    float target_speed = 0;
    if(cur_dist > SLOW_DOWN_DIST) {
        target_speed = APPROACH_SPEED;
    } else if(cur_dist > SAFE_DISTANCE) {
        target_speed = APPROACH_SPEED * (cur_dist - SAFE_DISTANCE)/(SLOW_DOWN_DIST - SAFE_DISTANCE);
    } else {
        target_speed = -0.3f; // 后退速度
    }
    
    // 转向控制
    float steer = 0;
    if(fabsf(error_x) > STEER_DEADZONE) {
        steer = STEER_KP*error_x + STEER_KD*error_d;
        steer = target_limit_float(steer, -MAX_STEER, MAX_STEER);
        
        // 距离相关增益
        float dist_gain = target_speed > 0 ? 
                        cur_dist/SLOW_DOWN_DIST : 
                        1.0f;
        steer *= dist_gain;
    }
    
    // 对准检测
    if(fabsf(error_x) < 30 && cur_dist < SAFE_DISTANCE*1.2) {
        if(++align_cnt > 50) { // 250ms稳定
            Stop_Motor_With_Kinematics();
            align_cnt = 0;
            return;
        }
    } else {
        align_cnt = 0;
    }
    
    // 执行运动
    Get_Target_Encoder(target_speed, steer);
}



/**************************************************************************
Function: Bluetooth_Control
Input   : none
Output  : none
函数功能：手机蓝牙控制
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//void Bluetooth_Control(void)
//{
//	if(Flag_Direction==0) Move_X=0,Move_Z=0;  			 						//停止
//	else if(Flag_Direction==1) Move_X=RC_Velocity,Move_Z=0;  					//前进
//	else if(Flag_Direction==2) Move_X=RC_Velocity,Move_Z=Pi/2;  	//右前
//	else if(Flag_Direction==3) Move_X=0,Move_Z=Pi/2;   				//向右
//	else if(Flag_Direction==4) Move_X=-RC_Velocity,Move_Z=Pi/2; 	//右后
//	else if(Flag_Direction==5) Move_X=-RC_Velocity,Move_Z=0;    				//后退
//	else if(Flag_Direction==6) Move_X=-RC_Velocity,Move_Z=-Pi/2; 	//左后
//	else if(Flag_Direction==7) Move_X=0,Move_Z=-Pi/2;      		 	//向左
//	else if(Flag_Direction==8) Move_X=RC_Velocity,Move_Z=-Pi/2;  	//左前
//	else Move_X=0,Move_Z=0; 
//	
//	if(Car_Num==Akm_Car)
//	{
//		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
//		//阿克曼结构小车转换为前轮转向角度
//		Move_Z=Move_Z*2/10; 
//	}
//	Move_X=Move_X/1000;     Move_Z=-Move_Z;
//}

/**************************************************************************
Function: PS2_Control
Input   : none
Output  : none
函数功能：PS2手柄控制
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//void PS2_Control(void)
//{
//	int LY,RX;									//手柄ADC的值
//	int Threshold=20; 							//阈值，忽略摇杆小幅度动作
//	static u8 Key1_Count = 0,Key2_Count = 0;	//用于控制读取摇杆的速度
//	//转化为128到-128的数值
//	LY=-(PS2_LY-128);//左边Y轴控制前进后退
//	RX=-(PS2_RX-128);//右边X轴控制转向

//	if(LY>-Threshold&&LY<Threshold)	LY=0;
//	if(RX>-Threshold&&RX<Threshold)	RX=0;		//忽略摇杆小幅度动作
//	
//	Move_X = (RC_Velocity/128)*LY;				//速度控制，力度表示速度大小
//	if(Car_Num == Akm_Car)						//阿克曼车转向控制，力度表示转向角度
//		Move_Z = -(RC_Turn_Velocity/128)*RX;	
//	else										//其他车型转向控制
//	{
//		if(Move_X>=0)
//			Move_Z = -(RC_Turn_Velocity/128)*RX;	//转向控制，力度表示转向速度
//		else
//			Move_Z = (RC_Turn_Velocity/128)*RX;
//	}
//	if (PS2_KEY == PSB_L1) 					 	//按下左1键加速（按键在顶上）
//	{	
//		if((++Key1_Count) == 20)				//调节按键反应速度
//		{
//			PS2_KEY = 0;
//			Key1_Count = 0;
//			if((RC_Velocity += X_Step)>MAX_RC_Velocity)				//前进最大速度1230
//				RC_Velocity = MAX_RC_Velocity;
//			if(Car_Num != Akm_Car)								//非阿克曼车可调节转向速度
//			{
//				if((RC_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)	//转向最大速度325
//					RC_Turn_Velocity = MAX_RC_Turn_Bias;
//			}
//		}
//	}
//	else if(PS2_KEY == PSB_R1) 					//按下右1键减速
//	{
//		if((++Key2_Count) == 20)
//		{
//			PS2_KEY = 0;
//			Key2_Count = 0;
//			if((RC_Velocity -= X_Step)<MINI_RC_Velocity)			//前后最小速度210
//				RC_Velocity = MINI_RC_Velocity;
//			
//			if(Car_Num != Akm_Car)								//非阿克曼车可调节转向速度
//			{
//				if((RC_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)//转向最小速度45
//				RC_Turn_Velocity = MINI_RC_Turn_Velocity;
//			}
//		}
//	}
//	else
//		Key2_Count = 0,Key2_Count = 0;			//读取到其他按键重新计数
//	Move_X=Move_X/1000;  Move_Z=-Move_Z;
//}
/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(void)
{
	
	 //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr; 
		OriginalEncoder.A=Read_Encoder(Encoder1);	
		OriginalEncoder.B=Read_Encoder(Encoder2);	

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Num)
		{
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break;
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break; 
			case Small_Tank_Car:Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break;
			case Big_Tank_Car:  Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break;
		}
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MotorA.Current_Encoder= Encoder_A_pr*Frequency*Perimeter/1560.0f;  
		MotorB.Current_Encoder= Encoder_B_pr*Frequency*Perimeter/1560.0f;   //1560=4*13*30=2（两路脉冲）*2（上下沿计数）*霍尔编码器13线*电机的减速比
//		MotorA.Current_Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Akm_wheelspacing//(4*13*30);  
//		MotorB.Current_Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Akm_wheelspacing/Encoder_precision;  
}
/**************************************************************************
Function: Drive_Motor
Input   : none
Output  : none
函数功能：运动学逆解
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
//阿克曼车Vz是舵机转向的角度(弧度制)
void Get_Target_Encoder(float Vx,float Vz)
{
  
 // 阿克曼转向约束
    const float MIN_TURN_RADIUS = 0.6f; // 最小转弯半径
  
    if(Car_Num == Akm_Car) {
        // 计算理论转向角
        float steer_angle = 0;
        if(fabsf(Vz) > 0.01f) {
            float R = Vx / Vz;
            if(fabsf(R) < MIN_TURN_RADIUS) {
                R = MIN_TURN_RADIUS * (R > 0 ? 1 : -1);
                Vz = Vx / R;
            }
            steer_angle = atanf(Akm_axlespacing / R);
        }
        
        // 转向角限幅
        steer_angle = target_limit_float(steer_angle, 
                                      -MAX_STEER, 
                                      MAX_STEER);
        
        // 平滑过渡
        static float last_angle = 0;
        float max_delta = 5.0f * Pi/180.0f * 0.005f; // 5度/秒
        steer_angle = last_angle + target_limit_float(steer_angle - last_angle, 
                                                    -max_delta, 
                                                    max_delta);
        last_angle = steer_angle;
        
        // 计算舵机PWM
        Servo_PWM = SERVO_INIT + (int)(steer_angle * 1800.0f/Pi);
    }
//	else if(Car_Num==Diff_Car)											//差速小车
//	{
//		  if(Vx<0) Vz=-Vz;
//	      else     Vz=Vz;
//			//Inverse kinematics //运动学逆解
//		   MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //计算出左轮的目标速度
//		   MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //计算出右轮的目标速度
//			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
//		   MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	       MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
//	}  
//	else if(Car_Num==Small_Tank_Car)
//	{
//		  if(Vx<0) Vz=-Vz;
//	      else     Vz=Vz;
//		  MotorA.Target_Encoder = Vx-Vz*Wheelspacing/2.0f;//计算出左轮的目标速度
//		  MotorB.Target_Encoder = Vx+Vz*Wheelspacing/2.0f;//计算出右轮的目标速度
//		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
//		  MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	      MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
//	}
//	else if(Car_Num==Big_Tank_Car)
//	{
//		  if(Vx<0) Vz=-Vz;
//	      else     Vz=Vz;
//		  MotorA.Target_Encoder = Vx-Vz*Wheelspacing/2.0f;//计算出左轮的目标速度
//		  MotorB.Target_Encoder = Vx+Vz*Wheelspacing/2.0f;//计算出右轮的目标速度
//		  MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	      MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
//	}
}
/**************************************************************************
Function: Get_Motor_PWM
Input   : none
Output  : none
函数功能：转换成驱动电机的PWM
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Motor_PWM(void)
{
	//计算左右电机对应的PWM
	MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
	MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
	if(Mode==Normal_Mode||Mode == Measure_Distance_Mode)
	{
		//滤波，使起步和停止稍微平滑一些
		MotorA.Motor_Pwm  = Mean_Filter_Left(MotorA.Motor_Pwm);
		MotorB.Motor_Pwm  = Mean_Filter_Right(MotorB.Motor_Pwm);
	}
	//限幅
	MotorA.Motor_Pwm  = PWM_Limit(MotorA.Motor_Pwm,PWM_MAX,PWM_MIN);
	MotorB.Motor_Pwm  = PWM_Limit(MotorB.Motor_Pwm,PWM_MAX,PWM_MIN);
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: Check whether it is abnormal
Input   : none
Output  : 1:Abnormal;0:Normal
函数功能：异常关闭电机
入口参数: 无 
返回  值：1：异常  0：正常
**************************************************************************/	 	
u8 Turn_Off(void)
{
	u8 temp = Normal;
	Flag_Stop = KEY2_STATE;			//读取按键2状态，按键2控制电机的开关
	if(Voltage<1000)				//电池电压低于10V关闭电机,LED灯快速闪烁
		LED_Flash(50),temp=Abnormal;
	else
		LED_Flash(200);				//每一秒闪一次，正常运行
	if(Flag_Stop)
		temp=Abnormal;
	return temp;			
}

/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
函数功能：数据滑动滤波
入口参数：数据
返回  值：滤波后的数据
**************************************************************************/
float Mean_Filter_Left(float data)
{
	u8 i;
	float Sum_Data = 0; 
	float Filter_Data;
	static  float Speed_Buf[FILTERING_TIMES]={0};
	for(i = 1 ; i<FILTERING_TIMES; i++)
	{
		Speed_Buf[i - 1] = Speed_Buf[i];
	}
	Speed_Buf[FILTERING_TIMES - 1] =data;

	for(i = 0 ; i < FILTERING_TIMES; i++)
	{
		Sum_Data += Speed_Buf[i];
	}
	Filter_Data = (s32)(Sum_Data / FILTERING_TIMES);
	return Filter_Data;
  
}
/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
函数功能：数据滑动滤波
入口参数：数据
返回  值：滤波后的数据
**************************************************************************/
float Mean_Filter_Right(float data)
{
	  u8 i;
	  float Sum_Data = 0; 
	  float Filter_Data;
	  static  float Speed_Buf[FILTERING_TIMES]={0};
	  for(i = 1 ; i<FILTERING_TIMES; i++)
	  {
		Speed_Buf[i - 1] = Speed_Buf[i];
	  }
	  Speed_Buf[FILTERING_TIMES - 1] =data;

	  for(i = 0 ; i < FILTERING_TIMES; i++)
	  {
		Sum_Data += Speed_Buf[i];
	  }
	  Filter_Data = (s32)(Sum_Data / FILTERING_TIMES);
	  return Filter_Data;
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
函数功能：雷达避障模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Avoid(void)
{
	int i = 0; 
	u8 calculation_angle_cnt = 0;	//用于判断100个点中需要做避障的点
	float angle_sum = 0;			//粗略计算障碍物位于左或者右
	u8 distance_count = 0;			//距离小于某值的计数
	int distance = 350;				//设定避障距离,默认是300
	if(Car_Num == Akm_Car)
		distance = 400;				//阿克曼车设定是400mm
	else if(Car_Num == Big_Tank_Car)
		distance = 500;				//大履带车设定是500mm
	for(i=0;i<lap_count;i++)				
	{
		if((Dataprocess[i].angle>310) || (Dataprocess[i].angle<50))
		{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<distance))	//距离小于350mm需要避障,只需要100度范围内点
			{
			  calculation_angle_cnt++;						 			//计算距离小于避障距离的点个数
				if(Dataprocess[i].angle<50)		
					angle_sum += Dataprocess[i].angle;
				else if(Dataprocess[i].angle>310)
					angle_sum += (Dataprocess[i].angle-360);	//310度到50度转化为-50度到50度
				if(Dataprocess[i].distance<200)				//记录小于200mm的点的计数
					distance_count++;
			}
	  }
	}
  
//  int corner=0;
//  if(calculation_angle_cnt > 80)
//  {
//     corner+=1;
//  }
  
	if(calculation_angle_cnt < 8)						//小于8点不需要避障，去除一些噪点
	{
		if((Move_X += 0.1)>=Aovid_Speed)							//避障的速度设定为260，逐渐增加到260可稍微平滑一些
			Move_X = 0.1;
		Move_Z = 0;										//不避障时不需要转弯
	}

//  else if(calculation_angle_cnt > 50)  // 小于8点不需要避障，去除一些噪点
//  {
//      Move_X = -Aovid_Speed;
//      
//Move_Z = -1;  // 随机向左转
//      delay_ms(3000);
//      corner = 0;
//  }

	else												//需要避障，简单地判断障碍物方位
	{
		if(Car_Num == Akm_Car)							//阿克曼车型有舵机，需要特殊处理
		{
			if(distance_count>8)						//距离小于避战距离
				Move_X = -Aovid_Speed,Move_Z = -Pi/4;				//往后退
        
			else
			{
				if((Move_X -= 0.1)<=(Aovid_Speed*0.5))					//避障时速度降到低速80
					Move_X = Aovid_Speed*0.5;
				if(angle_sum>0)							//障碍物偏右
					Move_Z = -Pi/4;						//每次转弯角度为PI/5，直到100度范围内无障碍物就停止
				else 									//偏左
					Move_Z = Pi/4;	
			}
		}
		else
		{
			if(distance_count>8)						//小于避战距离的时候
				Move_X = -0.5,Move_Z = 0;				//往后退
			else
			{
				if((Move_X -= 0.1)<=(Aovid_Speed*0.5))					//避障时速度降到低速度0.15
				Move_X = Aovid_Speed*0.5;
				if(angle_sum>0)							//障碍物偏右
				{	
					if(Car_Num == Diff_Car)				//每次转弯速度为X度，直到100度范围内无障碍物就停止
						Move_Z = -1;									
					else if(Car_Num == Small_Tank_Car)
						Move_Z = -1;	
					else
						Move_Z = -1;
				}
				else 									//偏左
				{
					if(Car_Num == Diff_Car)				//每次转弯速度为X度，直到100度范围内无障碍物就停止
						Move_Z = 1;									
					else if(Car_Num == Small_Tank_Car)
						Move_Z = 1;	
					else
						Move_Z = 1;
				}
			}
		}
	}
	Move_Z = -Move_Z;

}

/**************************************************************************
Function: Lidar_Follow
Input   : none
Output  : none
函数功能：雷达跟随模式
入口参数：无
返回  值：无
**************************************************************************/
float angle1 = 0;				//跟随的角度
u16 mini_distance1;
void Lidar_Follow(void)
{
	static u16 cnt = 0;
	int i;
	int calculation_angle_cnt = 0;
	static float angle = 0;				//跟随的角度
	static float last_angle = 0;		//
	u16 mini_distance = 65535;
	static u8 data_count = 0;			//用于滤除一写噪点的计数变量
	//需要找出跟随的那个点的角度
	for(i=0;i<lap_count;i++)
	{
		if(100<Dataprocess[i].distance && Dataprocess[i].distance<Follow_Distance)//1200范围内就需要跟随
		{
			calculation_angle_cnt++;
			if(Dataprocess[i].distance<mini_distance)//找出距离最小的点
			{
				mini_distance = Dataprocess[i].distance;
				angle = Dataprocess[i].angle;
			}
		}
	}
	if(angle>180)
		angle -= 360;				//0--360度转换成0--180；-180--0（顺时针）
	if(angle-last_angle>10 ||angle-last_angle<-10)	//做一定消抖，波动大于10度的需要做判断
	{
		if(++data_count == 60)		//连续60次采集到的值(300ms后)和上次的比大于10度，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//波动小于10度的可以直接认为是有效值
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(calculation_angle_cnt<6)		//在跟随范围内的点少于6个
	{
		
		if(cnt < 40)		 		//连续计数超40次没有要跟随的点，此时才是不用跟随
			cnt++;
		if(cnt >= 40)
		{
			Move_X = 0;				//速度为0
			Move_Z = 0;
		}
	}
	else
	{
		cnt = 0;
		if(Car_Num==Akm_Car)
		{

			if((((angle>15)&&(angle<180)) || ((angle>-180)&&angle<-15))&&(mini_distance<500))//阿卡曼车型处理车头不对着跟随物，相当于倒车一样，一次不对准，那后退再来对准
			{
				Move_X = -0.20;
			    Move_Z = -Follow_Turn_PID(last_angle,0);
			}
			else
			{
				Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //保持距离保持在400mm
				Move_Z = Follow_Turn_PID(last_angle,0);
			}
		    
		}
		else//其余车型
		{
			if((angle > 50 || angle < -50)&&(mini_distance>400))
			{
				Move_Z = -0.0298f*last_angle;  //角度差距过大直接快速转向
				Move_X = 0;                   //差速小车和履带小车可以实现原地转动
			}
			else
			{
				Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //保持距离保持在400mm
				Move_Z = Follow_Turn_PID(last_angle,0);		//转向PID，车头永远对着跟随物品
			}
	    }
	}
	Move_Z = target_limit_float(Move_Z,-Pi/6,Pi/6);   //限幅
	Move_X= target_limit_float(Move_X,-0.4,0.4); 
}

/**************************************************************************
函数功能：小车走直线模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_along_wall(void)
{
	static u32 target_distance=0;
	static int n = 0;

	u32 distance;
	
	for(int i=0;i<lap_count;i++)
	{
		if(Dataprocess[i].angle>Along_Angle && Dataprocess[i].angle<(Along_Angle+2))
		{
			if(n == 0)
			{
				target_distance = Dataprocess[i].distance;   //获取的第一个点作为目标距离
				n++;
			}
			if(Dataprocess[i].distance < target_distance+100)  //+100限制获取距离的范围值
			{
				distance = Dataprocess[i].distance;          //获取实时距离
			}
		}
	}
	Move_X = forward_velocity;  //初始速度
	Move_Z = -Along_Adjust_PID(distance, target_distance);
	if(Car_Num == Akm_Car)
	{
		Move_Z = target_limit_float(Move_Z,-Pi/4,Pi/4);   //限幅
    }
	else if(Car_Num == Diff_Car)
		Move_Z = target_limit_float(Move_Z,-Pi/5,Pi/5);   //限幅
}

/**************************************************************************
Function: Car_Perimeter_Init
Input   : none
Output  : none
函数功能：计算小车各轮子的周长
入口参数：无
返回  值：无
**************************************************************************/
void Car_Perimeter_Init(void)
{
	if(Car_Num == Diff_Car||Car_Num == Akm_Car)
	{
		Perimeter = Diff_Car_Wheel_diameter*Pi;
		Wheelspacing = Diff_wheelspacing;
	}		
	else if(Car_Num == Small_Tank_Car)
	{
		Perimeter = Small_Tank_WheelDiameter*Pi;
		Wheelspacing = Small_Tank_wheelspacing;
	}
	else
	{
		Perimeter = Big_Tank_WheelDiameter*Pi;
		Wheelspacing = Big_Tank_wheelspacing;
	}
}
/**************************************************************************
Function: Ultrasonic_Follow
Input   : none
Output  : none
函数功能：超声波跟随模式
入口参数：无
返回  值：无
**************************************************************************/
void Ultrasonic_Follow(void)		//超声波跟随，只能单方向跟随
{
	Move_Z = 0;
	Read_Distane();					//读取超声波的距离
	if(Distance1 < 200)				//距离小于200mm，退后
	{
		if((Move_X-=3) < -210)	
			Move_X = -210;			//给一210后退速度
	}
	else if( Distance1> 270 && Distance1 < 750)	//距离在270到750之间是需要跟随前进
	{
		if((Move_X+=3) > 210)					//速度逐渐增加，给前进速度
			Move_X = 210;
	}
	else
	{
		if(Move_X>0)
		{
			if((Move_X -= 20) < 0)				//速度逐渐减到0
				Move_X = 0;
		}
		else
		{
			if((Move_X+=20)>0)					//速度逐渐减到0
				Move_X = 0;
		}
	}
}


/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      //读取加速度、角速度、倾角
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
//		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
//		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
//		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/Pi;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/Pi;     //计算倾角，转换单位为度
		Gyro_X=Gyro_X/65.5;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
		Gyro_Y=Gyro_Y/65.5;                              //陀螺仪量程转换	
		if(way==2)		  	
		{
			Roll= -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//卡尔曼滤波
			Pitch = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(way==3) 
		{  
			Roll = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//互补滤波
			Pitch= -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
	}
}




