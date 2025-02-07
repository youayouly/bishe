
#include "control.h"
#include "Lidar.h"
#include "MPU6050.h"

float Move_X = 0,Move_Z = 0;						//Ŀ���ٶȺ�Ŀ��ת���ٶ�
float PWM_Left,PWM_Right;					//���ҵ��PWMֵ
float RC_Velocity,RC_Turn_Velocity;			//ң�ؿ��Ƶ��ٶ�
u8 Mode = 0;								//ģʽѡ��Ĭ������ͨ�Ŀ���ģʽ
Motor_parameter MotorA,MotorB;				//���ҵ����ر���
int Servo_PWM = SERVO_INIT;					//�����������ر���
u8 Lidar_Detect = Lidar_Detect_ON;			//���Ѳ��ģʽ�״����ϰ��Ĭ�Ͽ���
float CCD_Move_X = 0.3;						//CCDѲ���ٶ�
float ELE_Move_X = 0.3;						//���Ѳ���ٶ�
u8 Ros_count=0;
Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����   
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;



int16_t ball_x = 0, ball_y = 0;
uint8_t ball_detected = 0;
uint32_t ball_last_tick=0;
uint32_t sys_tick = 0;
uint8_t checksum = 0;

int16_t ball_angle = 0;
int16_t ball_distance = 0;

float last_error = 0.0f; // ���Ϊȫ�ֱ���
uint8_t stable_count = 0;
float filtered_x = BALL_CENTER_X;

float Vx = 0.0f, Vz = 0.0f;
float distance_ratio = 0.0f;

//ֹͣ����
// ��ǿ��ֹͣ����
void Stop_Motor_With_Kinematics(void) 
{
    // �˶�ѧ����
    Get_Target_Encoder(0.0f, 0.0f);
    Move_X = 0; 
    Move_Z = 0;
    
    // �����������
    MotorA.Target_Encoder = MotorA.Current_Encoder; // ���ֵ�ǰ������ֵ
    MotorB.Target_Encoder = MotorB.Current_Encoder;
    
    // ǿ��PWM���0
    Set_Pwm(0, 0);
    
    // �������п�����״̬
    Reset_Incremental_PI_Left();
    Reset_Incremental_PI_Right();
    MotorA.Current_Encoder = MotorB.Current_Encoder = 0;
    
    // ��ո���״̬

    last_error = 0.0f;
}
BallFilter ball_filter = {0};

// ������Ч�Լ��
uint8_t is_valid_data(int16_t x, int16_t dist) {
    return (x >= 0 && x < 640 && 
           dist > 10 && dist < 800 && // 10cm~8m
           (sys_tick - ball_filter.last_update) < 200); // 1�볬ʱ
}

// ��ͨ�˲�
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

// ����ȫ�ֱ�����ʵ���ڴ���䣩
OperationMode current_mode2 = LIDAR_AVOID; // ��ʼ��Ϊ����ģʽ


/**************************************************************************
Function: Control Function
Input   : none
Output  : none
�������ܣ�5ms��ʱ�жϿ��ƺ���
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
int TIMING_TIM_IRQHandler(void)
{
  
  static uint8_t stop_cnt = 0;
	static u8 Count_CCD = 0;								//����CCD����Ƶ��
	if(TIM_GetITStatus( TIMING_TIM, TIM_IT_Update) != RESET ) //������1 ==0
	{			
		TIM_ClearITPendingBit(TIMING_TIM , TIM_IT_Update);
    
    sys_tick++; // ÿ5ms����1
        // ...ԭ���߼�...
    
		Get_Velocity_From_Encoder();								//��ȡ���ұ�������ֵ��ת�����ٶ�
        Get_KeyVal();		 		
		if(delay_flag)												//50ms��ʱ
		{
			if(++delay_50==10) delay_50=0,delay_flag=0;            	//���������ṩ50ms�ľ�׼��ʱ����Ҫ��������λ��ʾ����
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
                            Lidar_Avoid();  // ����ԭ�б����߼�
                            Get_Target_Encoder(Move_X, Move_Z); // �����˶�ѧת��
                        }
                        break;
                        
                    case BALL_TRACKING:
                        
                        if(!ball_detected) {
                                current_mode2 = LIDAR_AVOID;
                                Stop_Motor_With_Kinematics();
                            } else {
                                Track_Ball();
                                
                                // ����ֹͣ�������ٶ�+λ�ã�
                                if(fabsf(MotorA.Current_Encoder) < 0.05f && 
                                   fabsf(MotorB.Current_Encoder) < 0.05f &&
                                   fabsf(ball_filter.x - BALL_CENTER_X) < 30 &&
                                   ball_filter.dist < SAFE_DISTANCE*1.1f) 
                                {
                                   
                                    if(++stop_cnt > 20) { // 100ms�ȶ�
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
    
    
//		else if(Mode == ELE_Line_Patrol_Mode)						//���Ѳ��ģʽ
//			ELE_Mode();
//		else														//CCDģʽ
//		{
//			if(++Count_CCD == 4)									//���ڿ���Ƶ�ʣ�4*5 = 20ms����һ��
//			{
//				Count_CCD = 0;
//				CCD_Mode();											
//			}
//			else if(Count_CCD>4)
//				Count_CCD = 0;
//		}			
		
//		else 
//			Ultrasonic_Follow();									//����������
    
//		Get_Target_Encoder(Move_X,Move_Z);							//�˶�ѧ���⣬ת���ɱ�������Ŀ���ٶ�
    
    //�м��������������
		if(Turn_Off()==Normal)										//������Ƿ�رգ���ѹ�Ƿ���
		{				
			Get_Motor_PWM();										//ת�������������pwm
		}
		else
		{
			MotorA.Motor_Pwm  = 0,MotorB.Motor_Pwm = 0,Servo_PWM = SERVO_INIT;
		}
		Set_Pwm(-MotorA.Motor_Pwm,MotorB.Motor_Pwm);				//�������
	}		 	
	return 0;
}



void Track_Ball(void) {
  
    static float last_error = 0;
    static uint8_t align_cnt = 0;
    
    // ������Ч�Լ��
    if(!is_valid_data(ball_x, ball_distance)) {
        Stop_Motor_With_Kinematics();
        return;
    }
    
    // �����˲�����
    update_ball_filter(ball_x, ball_distance);
    float cur_x = ball_filter.x;
    float cur_dist = ball_filter.dist;
    
    // �������
    float error_x = BALL_CENTER_X - cur_x;
    float error_d = error_x - last_error;
    last_error = error_x;
    
    // ������Ʋ���
    float target_speed = 0;
    if(cur_dist > SLOW_DOWN_DIST) {
        target_speed = APPROACH_SPEED;
    } else if(cur_dist > SAFE_DISTANCE) {
        target_speed = APPROACH_SPEED * (cur_dist - SAFE_DISTANCE)/(SLOW_DOWN_DIST - SAFE_DISTANCE);
    } else {
        target_speed = -0.3f; // �����ٶ�
    }
    
    // ת�����
    float steer = 0;
    if(fabsf(error_x) > STEER_DEADZONE) {
        steer = STEER_KP*error_x + STEER_KD*error_d;
        steer = target_limit_float(steer, -MAX_STEER, MAX_STEER);
        
        // �����������
        float dist_gain = target_speed > 0 ? 
                        cur_dist/SLOW_DOWN_DIST : 
                        1.0f;
        steer *= dist_gain;
    }
    
    // ��׼���
    if(fabsf(error_x) < 30 && cur_dist < SAFE_DISTANCE*1.2) {
        if(++align_cnt > 50) { // 250ms�ȶ�
            Stop_Motor_With_Kinematics();
            align_cnt = 0;
            return;
        }
    } else {
        align_cnt = 0;
    }
    
    // ִ���˶�
    Get_Target_Encoder(target_speed, steer);
}



/**************************************************************************
Function: Bluetooth_Control
Input   : none
Output  : none
�������ܣ��ֻ���������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//void Bluetooth_Control(void)
//{
//	if(Flag_Direction==0) Move_X=0,Move_Z=0;  			 						//ֹͣ
//	else if(Flag_Direction==1) Move_X=RC_Velocity,Move_Z=0;  					//ǰ��
//	else if(Flag_Direction==2) Move_X=RC_Velocity,Move_Z=Pi/2;  	//��ǰ
//	else if(Flag_Direction==3) Move_X=0,Move_Z=Pi/2;   				//����
//	else if(Flag_Direction==4) Move_X=-RC_Velocity,Move_Z=Pi/2; 	//�Һ�
//	else if(Flag_Direction==5) Move_X=-RC_Velocity,Move_Z=0;    				//����
//	else if(Flag_Direction==6) Move_X=-RC_Velocity,Move_Z=-Pi/2; 	//���
//	else if(Flag_Direction==7) Move_X=0,Move_Z=-Pi/2;      		 	//����
//	else if(Flag_Direction==8) Move_X=RC_Velocity,Move_Z=-Pi/2;  	//��ǰ
//	else Move_X=0,Move_Z=0; 
//	
//	if(Car_Num==Akm_Car)
//	{
//		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
//		//�������ṹС��ת��Ϊǰ��ת��Ƕ�
//		Move_Z=Move_Z*2/10; 
//	}
//	Move_X=Move_X/1000;     Move_Z=-Move_Z;
//}

/**************************************************************************
Function: PS2_Control
Input   : none
Output  : none
�������ܣ�PS2�ֱ�����
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//void PS2_Control(void)
//{
//	int LY,RX;									//�ֱ�ADC��ֵ
//	int Threshold=20; 							//��ֵ������ҡ��С���ȶ���
//	static u8 Key1_Count = 0,Key2_Count = 0;	//���ڿ��ƶ�ȡҡ�˵��ٶ�
//	//ת��Ϊ128��-128����ֵ
//	LY=-(PS2_LY-128);//���Y�����ǰ������
//	RX=-(PS2_RX-128);//�ұ�X�����ת��

//	if(LY>-Threshold&&LY<Threshold)	LY=0;
//	if(RX>-Threshold&&RX<Threshold)	RX=0;		//����ҡ��С���ȶ���
//	
//	Move_X = (RC_Velocity/128)*LY;				//�ٶȿ��ƣ����ȱ�ʾ�ٶȴ�С
//	if(Car_Num == Akm_Car)						//��������ת����ƣ����ȱ�ʾת��Ƕ�
//		Move_Z = -(RC_Turn_Velocity/128)*RX;	
//	else										//��������ת�����
//	{
//		if(Move_X>=0)
//			Move_Z = -(RC_Turn_Velocity/128)*RX;	//ת����ƣ����ȱ�ʾת���ٶ�
//		else
//			Move_Z = (RC_Turn_Velocity/128)*RX;
//	}
//	if (PS2_KEY == PSB_L1) 					 	//������1�����٣������ڶ��ϣ�
//	{	
//		if((++Key1_Count) == 20)				//���ڰ�����Ӧ�ٶ�
//		{
//			PS2_KEY = 0;
//			Key1_Count = 0;
//			if((RC_Velocity += X_Step)>MAX_RC_Velocity)				//ǰ������ٶ�1230
//				RC_Velocity = MAX_RC_Velocity;
//			if(Car_Num != Akm_Car)								//�ǰ��������ɵ���ת���ٶ�
//			{
//				if((RC_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)	//ת������ٶ�325
//					RC_Turn_Velocity = MAX_RC_Turn_Bias;
//			}
//		}
//	}
//	else if(PS2_KEY == PSB_R1) 					//������1������
//	{
//		if((++Key2_Count) == 20)
//		{
//			PS2_KEY = 0;
//			Key2_Count = 0;
//			if((RC_Velocity -= X_Step)<MINI_RC_Velocity)			//ǰ����С�ٶ�210
//				RC_Velocity = MINI_RC_Velocity;
//			
//			if(Car_Num != Akm_Car)								//�ǰ��������ɵ���ת���ٶ�
//			{
//				if((RC_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)//ת����С�ٶ�45
//				RC_Turn_Velocity = MINI_RC_Turn_Velocity;
//			}
//		}
//	}
//	else
//		Key2_Count = 0,Key2_Count = 0;			//��ȡ�������������¼���
//	Move_X=Move_X/1000;  Move_Z=-Move_Z;
//}
/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
�������ܣ���ȡ��������ת�����ٶ�
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void Get_Velocity_From_Encoder(void)
{
	
	 //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr; 
		OriginalEncoder.A=Read_Encoder(Encoder1);	
		OriginalEncoder.B=Read_Encoder(Encoder2);	

	  //Decide the encoder numerical polarity according to different car models
		//���ݲ�ͬС���ͺž�����������ֵ����
		switch(Car_Num)
		{
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break;
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break; 
			case Small_Tank_Car:Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break;
			case Big_Tank_Car:  Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;break;
		}
		//The encoder converts the raw data to wheel speed in m/s
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MotorA.Current_Encoder= Encoder_A_pr*Frequency*Perimeter/1560.0f;  
		MotorB.Current_Encoder= Encoder_B_pr*Frequency*Perimeter/1560.0f;   //1560=4*13*30=2����·���壩*2�������ؼ�����*����������13��*����ļ��ٱ�
//		MotorA.Current_Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Akm_wheelspacing//(4*13*30);  
//		MotorB.Current_Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Akm_wheelspacing/Encoder_precision;  
}
/**************************************************************************
Function: Drive_Motor
Input   : none
Output  : none
�������ܣ��˶�ѧ���
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//�˶�ѧ��⣬��x��y���ٶȵõ����������ٶ�,Vx��m/s,Vz��λ�Ƕ�/s(�Ƕ���)
//��������Vz�Ƕ��ת��ĽǶ�(������)
void Get_Target_Encoder(float Vx,float Vz)
{
  
 // ������ת��Լ��
    const float MIN_TURN_RADIUS = 0.6f; // ��Сת��뾶
  
    if(Car_Num == Akm_Car) {
        // ��������ת���
        float steer_angle = 0;
        if(fabsf(Vz) > 0.01f) {
            float R = Vx / Vz;
            if(fabsf(R) < MIN_TURN_RADIUS) {
                R = MIN_TURN_RADIUS * (R > 0 ? 1 : -1);
                Vz = Vx / R;
            }
            steer_angle = atanf(Akm_axlespacing / R);
        }
        
        // ת����޷�
        steer_angle = target_limit_float(steer_angle, 
                                      -MAX_STEER, 
                                      MAX_STEER);
        
        // ƽ������
        static float last_angle = 0;
        float max_delta = 5.0f * Pi/180.0f * 0.005f; // 5��/��
        steer_angle = last_angle + target_limit_float(steer_angle - last_angle, 
                                                    -max_delta, 
                                                    max_delta);
        last_angle = steer_angle;
        
        // ������PWM
        Servo_PWM = SERVO_INIT + (int)(steer_angle * 1800.0f/Pi);
    }
//	else if(Car_Num==Diff_Car)											//����С��
//	{
//		  if(Vx<0) Vz=-Vz;
//	      else     Vz=Vz;
//			//Inverse kinematics //�˶�ѧ���
//		   MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //��������ֵ�Ŀ���ٶ�
//		   MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //��������ֵ�Ŀ���ٶ�
//			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
//		   MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	       MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
//	}  
//	else if(Car_Num==Small_Tank_Car)
//	{
//		  if(Vx<0) Vz=-Vz;
//	      else     Vz=Vz;
//		  MotorA.Target_Encoder = Vx-Vz*Wheelspacing/2.0f;//��������ֵ�Ŀ���ٶ�
//		  MotorB.Target_Encoder = Vx+Vz*Wheelspacing/2.0f;//��������ֵ�Ŀ���ٶ�
//		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
//		  MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	      MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
//	}
//	else if(Car_Num==Big_Tank_Car)
//	{
//		  if(Vx<0) Vz=-Vz;
//	      else     Vz=Vz;
//		  MotorA.Target_Encoder = Vx-Vz*Wheelspacing/2.0f;//��������ֵ�Ŀ���ٶ�
//		  MotorB.Target_Encoder = Vx+Vz*Wheelspacing/2.0f;//��������ֵ�Ŀ���ٶ�
//		  MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	      MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
//	}
}
/**************************************************************************
Function: Get_Motor_PWM
Input   : none
Output  : none
�������ܣ�ת�������������PWM
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void Get_Motor_PWM(void)
{
	//�������ҵ����Ӧ��PWM
	MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
	MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
	if(Mode==Normal_Mode||Mode == Measure_Distance_Mode)
	{
		//�˲���ʹ�𲽺�ֹͣ��΢ƽ��һЩ
		MotorA.Motor_Pwm  = Mean_Filter_Left(MotorA.Motor_Pwm);
		MotorB.Motor_Pwm  = Mean_Filter_Right(MotorB.Motor_Pwm);
	}
	//�޷�
	MotorA.Motor_Pwm  = PWM_Limit(MotorA.Motor_Pwm,PWM_MAX,PWM_MIN);
	MotorB.Motor_Pwm  = PWM_Limit(MotorB.Motor_Pwm,PWM_MAX,PWM_MIN);
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
�������ܣ�����PWM��ֵ
��ڲ���: IN���������  max���޷����ֵ  min���޷���Сֵ 
����  ֵ���޷����ֵ
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
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
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
�������ܣ��쳣�رյ��
��ڲ���: �� 
����  ֵ��1���쳣  0������
**************************************************************************/	 	
u8 Turn_Off(void)
{
	u8 temp = Normal;
	Flag_Stop = KEY2_STATE;			//��ȡ����2״̬������2���Ƶ���Ŀ���
	if(Voltage<1000)				//��ص�ѹ����10V�رյ��,LED�ƿ�����˸
		LED_Flash(50),temp=Abnormal;
	else
		LED_Flash(200);				//ÿһ����һ�Σ���������
	if(Flag_Stop)
		temp=Abnormal;
	return temp;			
}

/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
�������ܣ����ݻ����˲�
��ڲ���������
����  ֵ���˲��������
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
�������ܣ����ݻ����˲�
��ڲ���������
����  ֵ���˲��������
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
�������ܣ��״����ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Avoid(void)
{
	int i = 0; 
	u8 calculation_angle_cnt = 0;	//�����ж�100��������Ҫ�����ϵĵ�
	float angle_sum = 0;			//���Լ����ϰ���λ���������
	u8 distance_count = 0;			//����С��ĳֵ�ļ���
	int distance = 350;				//�趨���Ͼ���,Ĭ����300
	if(Car_Num == Akm_Car)
		distance = 400;				//���������趨��400mm
	else if(Car_Num == Big_Tank_Car)
		distance = 500;				//���Ĵ����趨��500mm
	for(i=0;i<lap_count;i++)				
	{
		if((Dataprocess[i].angle>310) || (Dataprocess[i].angle<50))
		{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<distance))	//����С��350mm��Ҫ����,ֻ��Ҫ100�ȷ�Χ�ڵ�
			{
			  calculation_angle_cnt++;						 			//�������С�ڱ��Ͼ���ĵ����
				if(Dataprocess[i].angle<50)		
					angle_sum += Dataprocess[i].angle;
				else if(Dataprocess[i].angle>310)
					angle_sum += (Dataprocess[i].angle-360);	//310�ȵ�50��ת��Ϊ-50�ȵ�50��
				if(Dataprocess[i].distance<200)				//��¼С��200mm�ĵ�ļ���
					distance_count++;
			}
	  }
	}
  
//  int corner=0;
//  if(calculation_angle_cnt > 80)
//  {
//     corner+=1;
//  }
  
	if(calculation_angle_cnt < 8)						//С��8�㲻��Ҫ���ϣ�ȥ��һЩ���
	{
		if((Move_X += 0.1)>=Aovid_Speed)							//���ϵ��ٶ��趨Ϊ260�������ӵ�260����΢ƽ��һЩ
			Move_X = 0.1;
		Move_Z = 0;										//������ʱ����Ҫת��
	}

//  else if(calculation_angle_cnt > 50)  // С��8�㲻��Ҫ���ϣ�ȥ��һЩ���
//  {
//      Move_X = -Aovid_Speed;
//      
//Move_Z = -1;  // �������ת
//      delay_ms(3000);
//      corner = 0;
//  }

	else												//��Ҫ���ϣ��򵥵��ж��ϰ��﷽λ
	{
		if(Car_Num == Akm_Car)							//�����������ж������Ҫ���⴦��
		{
			if(distance_count>8)						//����С�ڱ�ս����
				Move_X = -Aovid_Speed,Move_Z = -Pi/4;				//������
        
			else
			{
				if((Move_X -= 0.1)<=(Aovid_Speed*0.5))					//����ʱ�ٶȽ�������80
					Move_X = Aovid_Speed*0.5;
				if(angle_sum>0)							//�ϰ���ƫ��
					Move_Z = -Pi/4;						//ÿ��ת��Ƕ�ΪPI/5��ֱ��100�ȷ�Χ�����ϰ����ֹͣ
				else 									//ƫ��
					Move_Z = Pi/4;	
			}
		}
		else
		{
			if(distance_count>8)						//С�ڱ�ս�����ʱ��
				Move_X = -0.5,Move_Z = 0;				//������
			else
			{
				if((Move_X -= 0.1)<=(Aovid_Speed*0.5))					//����ʱ�ٶȽ������ٶ�0.15
				Move_X = Aovid_Speed*0.5;
				if(angle_sum>0)							//�ϰ���ƫ��
				{	
					if(Car_Num == Diff_Car)				//ÿ��ת���ٶ�ΪX�ȣ�ֱ��100�ȷ�Χ�����ϰ����ֹͣ
						Move_Z = -1;									
					else if(Car_Num == Small_Tank_Car)
						Move_Z = -1;	
					else
						Move_Z = -1;
				}
				else 									//ƫ��
				{
					if(Car_Num == Diff_Car)				//ÿ��ת���ٶ�ΪX�ȣ�ֱ��100�ȷ�Χ�����ϰ����ֹͣ
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
�������ܣ��״����ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
float angle1 = 0;				//����ĽǶ�
u16 mini_distance1;
void Lidar_Follow(void)
{
	static u16 cnt = 0;
	int i;
	int calculation_angle_cnt = 0;
	static float angle = 0;				//����ĽǶ�
	static float last_angle = 0;		//
	u16 mini_distance = 65535;
	static u8 data_count = 0;			//�����˳�һд���ļ�������
	//��Ҫ�ҳ�������Ǹ���ĽǶ�
	for(i=0;i<lap_count;i++)
	{
		if(100<Dataprocess[i].distance && Dataprocess[i].distance<Follow_Distance)//1200��Χ�ھ���Ҫ����
		{
			calculation_angle_cnt++;
			if(Dataprocess[i].distance<mini_distance)//�ҳ�������С�ĵ�
			{
				mini_distance = Dataprocess[i].distance;
				angle = Dataprocess[i].angle;
			}
		}
	}
	if(angle>180)
		angle -= 360;				//0--360��ת����0--180��-180--0��˳ʱ�룩
	if(angle-last_angle>10 ||angle-last_angle<-10)	//��һ����������������10�ȵ���Ҫ���ж�
	{
		if(++data_count == 60)		//����60�βɼ�����ֵ(300ms��)���ϴεıȴ���10�ȣ���ʱ������Ϊ����Чֵ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//����С��10�ȵĿ���ֱ����Ϊ����Чֵ
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(calculation_angle_cnt<6)		//�ڸ��淶Χ�ڵĵ�����6��
	{
		
		if(cnt < 40)		 		//����������40��û��Ҫ����ĵ㣬��ʱ���ǲ��ø���
			cnt++;
		if(cnt >= 40)
		{
			Move_X = 0;				//�ٶ�Ϊ0
			Move_Z = 0;
		}
	}
	else
	{
		cnt = 0;
		if(Car_Num==Akm_Car)
		{

			if((((angle>15)&&(angle<180)) || ((angle>-180)&&angle<-15))&&(mini_distance<500))//���������ʹ���ͷ�����Ÿ�����൱�ڵ���һ����һ�β���׼���Ǻ���������׼
			{
				Move_X = -0.20;
			    Move_Z = -Follow_Turn_PID(last_angle,0);
			}
			else
			{
				Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //���־��뱣����400mm
				Move_Z = Follow_Turn_PID(last_angle,0);
			}
		    
		}
		else//���೵��
		{
			if((angle > 50 || angle < -50)&&(mini_distance>400))
			{
				Move_Z = -0.0298f*last_angle;  //�ǶȲ�����ֱ�ӿ���ת��
				Move_X = 0;                   //����С�����Ĵ�С������ʵ��ԭ��ת��
			}
			else
			{
				Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //���־��뱣����400mm
				Move_Z = Follow_Turn_PID(last_angle,0);		//ת��PID����ͷ��Զ���Ÿ�����Ʒ
			}
	    }
	}
	Move_Z = target_limit_float(Move_Z,-Pi/6,Pi/6);   //�޷�
	Move_X= target_limit_float(Move_X,-0.4,0.4); 
}

/**************************************************************************
�������ܣ�С����ֱ��ģʽ
��ڲ�������
����  ֵ����
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
				target_distance = Dataprocess[i].distance;   //��ȡ�ĵ�һ������ΪĿ�����
				n++;
			}
			if(Dataprocess[i].distance < target_distance+100)  //+100���ƻ�ȡ����ķ�Χֵ
			{
				distance = Dataprocess[i].distance;          //��ȡʵʱ����
			}
		}
	}
	Move_X = forward_velocity;  //��ʼ�ٶ�
	Move_Z = -Along_Adjust_PID(distance, target_distance);
	if(Car_Num == Akm_Car)
	{
		Move_Z = target_limit_float(Move_Z,-Pi/4,Pi/4);   //�޷�
    }
	else if(Car_Num == Diff_Car)
		Move_Z = target_limit_float(Move_Z,-Pi/5,Pi/5);   //�޷�
}

/**************************************************************************
Function: Car_Perimeter_Init
Input   : none
Output  : none
�������ܣ�����С�������ӵ��ܳ�
��ڲ�������
����  ֵ����
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
�������ܣ�����������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Ultrasonic_Follow(void)		//���������棬ֻ�ܵ��������
{
	Move_Z = 0;
	Read_Distane();					//��ȡ�������ľ���
	if(Distance1 < 200)				//����С��200mm���˺�
	{
		if((Move_X-=3) < -210)	
			Move_X = -210;			//��һ210�����ٶ�
	}
	else if( Distance1> 270 && Distance1 < 750)	//������270��750֮������Ҫ����ǰ��
	{
		if((Move_X+=3) > 210)					//�ٶ������ӣ���ǰ���ٶ�
			Move_X = 210;
	}
	else
	{
		if(Move_X>0)
		{
			if((Move_X -= 20) < 0)				//�ٶ��𽥼���0
				Move_X = 0;
		}
		else
		{
			if((Move_X+=20)>0)					//�ٶ��𽥼���0
				Move_X = 0;
		}
	}
}


/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�	
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      //��ȡ���ٶȡ����ٶȡ����
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //��������ת��
//		if(Accel_X>32768) Accel_X-=65536;                //��������ת��
//		if(Accel_Y>32768) Accel_Y-=65536;                //��������ת��
//		if(Accel_Z>32768) Accel_Z-=65536;                //��������ת��
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/Pi;     //������ǣ�ת����λΪ��	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/Pi;     //������ǣ�ת����λΪ��
		Gyro_X=Gyro_X/65.5;                              //����������ת�������̡�2000��/s��Ӧ������16.4���ɲ��ֲ�
		Gyro_Y=Gyro_Y/65.5;                              //����������ת��	
		if(way==2)		  	
		{
			Roll= -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//�������˲�
			Pitch = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(way==3) 
		{  
			Roll = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//�����˲�
			Pitch= -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
	}
}




