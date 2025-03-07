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

int16_t stable_count = 0;
const int STABLE_THRESHOLD = 5; // 稳定阈值，连续5次检测到球稳定
const int SERVO_GRAB_POSITION = 500; // 舵机抓取位置
const int SERVO_RELEASE_POSITION = 1700; // 舵机释放位置
uint32_t grab_start_time = 0;


float servo_action_counter = 0;

// 舵机动作参数
#define SERVO_GRAB_STEPS   1000 // 抓取动作持续计数（5ms*200=1s）
#define SERVO_RELEASE_STEPS 800 // 释放动作持续计数
// 抓取动作持续计数（5ms*200=1s）

//停止函数
void Stop_Motor_With_Kinematics(void) {
    // 调用运动学逆解函数，速度和角度均为0
    Get_Target_Encoder(0.0f, 0.0f); // Vx=0, Vz=0
  
  
    Move_X = 0;
    Move_Z = 0;
    MotorA.Target_Encoder = 0;
    MotorB.Target_Encoder = 0;
    // 直接设置PWM为0
    Set_Pwm(0, 0);
    // 调试输出
}

// 定义全局变量（实际内存分配）
OperationMode current_mode2 = LIDAR_AVOID; // 初始化为避障模式
// 定义一个全局或静态变量，记录连续未检测到球的次数
static int ball_not_detect_count = 0;

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
  
  
	static u8 Count_CCD = 0;								//调节CCD控制频率
	if(TIM_GetITStatus( TIMING_TIM, TIM_IT_Update) != RESET ) //不等于1 ==0
	{			
		TIM_ClearITPendingBit(TIMING_TIM , TIM_IT_Update);
    
    sys_tick++; // 每5ms自增1
        // ...原有逻辑...
    
            // 检查是否超时未接收到球数据 两次识别到球之间的时间 而且可以重置
//        if ((sys_tick - ball_last_tick) > BALL_TIMEOUT) {
//            // 超时了：认为球丢失，停止车辆，切换到避障模式
//            //Stop_Motor_With_Kinematics();
//            // 可以选择更新模式变量，比如：
//            current_mode2 = LIDAR_AVOID;
//        }
//        
		Get_Velocity_From_Encoder();								//读取左右编码器的值且转换成速度
        Get_KeyVal();		 		
    
    //这段注释了没有定时，按键也失效
		if(delay_flag)												//50ms延时
		{
			if(++delay_50==10) delay_50=0,delay_flag=0;            	//给主函数提供50ms的精准延时，主要是用于上位机示波器
			else if(delay_50>10)	delay_50=0,delay_flag=0;
		}
    
    switch(Mode) {
            case Normal_Mode:
                if(++Ros_count == 10) {
                    Get_Angle(2);
                    Ros_count = 0;
                }
                break;
             case Lidar_Avoid_Mode:
//                if (current_mode2 != BALL_GRABBING) {
                    switch (current_mode2) {
                        case LIDAR_AVOID:
                            if (ball_detected) {
                              if((current_mode2 != BALL_STABLE) && (current_mode2 != BALL_GRABBING)){
                                current_mode2 = BALL_TRACKING;
                                }
                                ball_last_tick = sys_tick;
                            } else {
                                Lidar_Avoid();
                                Get_Target_Encoder(Move_X, Move_Z);
                            }
                            break;
                        case BALL_TRACKING:
                            if (!ball_detected) {
                               ball_not_detect_count++;  // 累加未检测到次数
                                 if (ball_not_detect_count >= 4) {
                                current_mode2 = LIDAR_AVOID;
                                //Stop_Motor_With_Kinematics();
                                 }
                            } else {
                                      // 检测到球，重置计数器
                                ball_not_detect_count = 0;
                                Track_Ball();
                              
                                  if (stable_count >= STABLE_THRESHOLD) {
                                    Stop_Motor_With_Kinematics();
                                    current_mode2 = BALL_STABLE;
                                }
//                                if ((abs(ball_x - BALL_CENTER_X) < BALL_DEADZONE) && (abs(ball_distance - TARGET_DISTANCE) < DIST_DEADZONE)) {
//                                    Stop_Motor_With_Kinematics();
//                                    current_mode2 = BALL_STABLE;
//                                }
                                ball_last_tick = sys_tick;
                            }
                            break;
              
                        case BALL_STABLE:
                                // 进入稳定状态后立即启动抓取
                                SERVO3_INIT = SERVO_GRAB_POSITION; // 600
                                Set_Servo_PWM(SERVO3_INIT);
                                servo_action_counter = SERVO_GRAB_STEPS; // 初始化计数器
                                current_mode2 = BALL_GRABBING; // 进入抓取动作状态
                                break;

                        case BALL_GRABBING:
                            if (servo_action_counter > 0) {
                                servo_action_counter--; // 每次中断递减
                            } 
                            if(servo_action_counter<=500)
                              {
                                // 抓取完成，切换到释放状态
                          
                                SERVO3_INIT = SERVO_RELEASE_POSITION; // 1700
                                Set_Servo_PWM(SERVO3_INIT);
                               
                                //current_mode2 = LIDAR_AVOID;
                            }
                            
                            if(servo_action_counter<=0){
                                 //servo_action_counter = SERVO_RELEASE_STEPS;
                                 current_mode2 = LIDAR_AVOID;
                            }
                            
                            break;
                    }
//                }
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

// 全局变量
float prev_error_distance = 0.0f;
float integral_distance = 0.0f;

float integral_angle = 0.0f;
float prev_error_angle = 0.0f;

//均值滤波 放置车前后波动
#define FILTER_SIZE 5
float distance_buffer[FILTER_SIZE] = {0};
int buffer_index = 0;

float FilterDistance(float new_distance) {
    distance_buffer[buffer_index] = new_distance;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;

    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += distance_buffer[i];
    }
    return sum / FILTER_SIZE;
}

//差分阈值
float previous_distance = 0.0f;
float threshold = 300.0f; // 根据实际情况设置阈值

float filter_spike(float new_distance) {
    if (fabs(new_distance - previous_distance) > threshold) {
        // 突变值，忽略
        return previous_distance;
    } else {
        previous_distance = new_distance;
        return new_distance;
    }
}

void Track_Ball(void) {
    if (ball_angle > 32767) {
        ball_angle = ball_angle - 65536;
    }
    
    const float Kp = 0.001f;
    const float Ki = 0.002f;
    const float Kd = 0.015f;
    const float Kp_angle = 0.0001f;
    const float Ki_angle = 0.00005f;
    const float Kd_angle = 0.005f;
    const float dt = 0.005f;
    
    const float MAX_FORWARD_SPEED = 0.15f;
    const float MAX_BACKWARD_SPEED = 0.10f;
    const float MIN_SPEED = 0.02f;

    
    float error_distance = ball_distance - TARGET_DISTANCE;
    float angle_weight = 1.0f;
    if (ball_distance > TARGET_DISTANCE * 1.4f) {
        angle_weight = 0.8f;
    } else if (ball_distance < TARGET_DISTANCE * 0.8f) {
        angle_weight = 0.5f;
    }
    float error_angle = ball_angle * angle_weight;
    
    integral_distance += error_distance * dt;
    if (fabs(error_distance) > 0.5f * TARGET_DISTANCE)
        integral_distance = 0.0f;
    integral_angle += error_angle * dt;
    if (fabs(error_angle) > 0.5f * BALL_CENTER_X)
        integral_angle = 0.0f;
    
    float derivative_distance = (error_distance - prev_error_distance) / dt;
    prev_error_distance = error_distance;
    float derivative_angle = (error_angle - prev_error_angle) / dt;
    prev_error_angle = error_angle;
    
    float pid_output_x = Kp * error_distance + Ki * integral_distance + Kd * derivative_distance;
    Vx = pid_output_x;
    Vz = -(Kp_angle * error_angle + Ki_angle * integral_angle + Kd_angle * derivative_angle);
    
    float dynamic_max_speed = MAX_FORWARD_SPEED;
    if (ball_distance < 500) {
        dynamic_max_speed = MAX_FORWARD_SPEED * (ball_distance / 500.0f);
        dynamic_max_speed = fmaxf(MIN_SPEED, dynamic_max_speed);
    }
    
    Vx = fminf(dynamic_max_speed, fmaxf(-MAX_BACKWARD_SPEED, Vx));
    Vz = fminf(TURN_SPEED, fmaxf(-TURN_SPEED, Vz));
    
    int error_x = ball_x - BALL_CENTER_X;
    int error_y = ball_y - BALL_CENTER_Y;
    
         if (abs(error_x) > 40) {
        Vx = -0.05f;  // 小速后退调整角度
        Vz = -Vz;
        Vx = fminf(Vx, MAX_FORWARD_SPEED);
    }
         
    // 新增：捡球后停止并重置
    if (ball_distance < 450) {  // 假设 0.20 米为捡球阈值
      
       Vx = fminf(Vx, 0.05f * MAX_FORWARD_SPEED);
        stable_count = 0;  // 重置稳定计数
    } 
//    else if (ball_distance == 0 && ball_x == 0 && ball_y == 0) {  // 无目标时停止
//        Vx = 0.0f;
//        Vz = 0.0f;
//    } 

   else if (error_distance < 100 && error_distance > 40) {
        Vx = fminf(Vx, 0.2f * MAX_FORWARD_SPEED);
    } else if (error_distance <= 40 && error_distance >= 0) {
        Vx = fminf(Vx, 0.1f * MAX_FORWARD_SPEED);
    }
    

        
    if ((abs(error_x) < ANGLE_DEADZONE) && (abs(error_y) < DIST_DEADZONE))
        stable_count++;
    else
        stable_count = 0;
    
    Move_X = Vx;
    Move_Z = Vz;
    Get_Target_Encoder(Vx, Vz);
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
	float MotorA_Velocity,MotorB_Velocity;
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅
	if(Car_Num==Akm_Car)							//阿克曼车
	{
		//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, ratio=640.62, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Akm_axlespacing/tan(AngleR)-0.5f*Akm_wheelspacing;
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MotorA.Target_Encoder = Vx*(R-0.081f)/R;
				MotorB.Target_Encoder = Vx*(R+0.081f)/R;			
			}
			else 
			{
				MotorA.Target_Encoder = Vx;
				MotorB.Target_Encoder = Vx;
			}

			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度  根据角度算出pwm数值
			Angle_Servo = -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Servo_PWM=SERVO_INIT + (Angle_Servo - 1.572f)*ratio;

			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MotorA.Target_Encoder=target_limit_float(MotorA.Target_Encoder,-amplitude,amplitude); 
			MotorB.Target_Encoder=target_limit_float(MotorB.Target_Encoder,-amplitude,amplitude); 
			Servo_PWM=target_limit_int(Servo_PWM,800,2200);	//Servo PWM value limit //舵机PWM值限幅

	}

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
			Move_X = 0.10;
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
