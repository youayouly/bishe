#ifndef __CONTROL_H
#define __CONTROL_H

#include "Lidar.h"
#include "Header.h"

//����
// �ڶ�ʱ���ж���ά��ʱ���������5ms����һ�Σ�
extern uint32_t sys_tick;
#define BALL_TIMEOUT 200  // 200*5ms=1����������ΪĿ�궪ʧ

// ��ȡ��ǰʱ�䣨��λ��ms��
#define GET_TICK() (sys_tick * 5)

// �������Ϻ�׷��С����ͷ�ļ�����״̬��ȫ�ֱ���  ״̬������
typedef enum {
    LIDAR_AVOID,
    BALL_TRACKING
} OperationMode;


extern OperationMode current_mode2;

// ��������ر���
extern int16_t ball_x , ball_y ;
extern uint8_t ball_detected ;
extern int16_t ball_angle;      // �Ƕ�
extern int16_t ball_distance;   // ����
extern uint8_t checksum;


extern uint32_t ball_last_tick;

extern uint8_t ball_detected_counter;

#define BALL_TIMEOUT 200 // 200*5ms=1�볬ʱ
//-----------��control.h����Ӻ궨��-----------
#define BALL_CENTER_X        320     // ͼ������X����
#define BALL_DEADZONE_X      10      // ת������
#define STOP_DISTANCE        0.38f    // ֹͣ����(��)
#define TURN_SPEED           0.15f    // ת���ٶ�
#define FORWARD_SPEED        0.15f    // ǰ���ٶ�

#define MIN_TRACK_DISTANCE   0.4f    // ��С׷�پ���(��)
#define MAX_TRACK_DISTANCE   2.0f    // �����Ч���� 
#define MAX_TRACK_SPEED      0.2f    // ���ǰ���ٶ�
#define MAX_TURN_RATE        1.0f    // ���ת���ٶ�
#define TRACK_P_GAIN        0.008f   // ת�����ϵ��
#define SAFE_DISTANCE       0.5f     // ��ȫֹͣ����

// �������Ʋ���
#define KP 0.005f       // ת�����ϵ��
#define TARGET_DISTANCE 0.4f  // Ŀ����پ��� (��)

#define BALL_CENTER_Y      240
#define BALL_DEADZONE      20
// ��Track_Ball�����м��뾫ȷ��������
#define ANGLE_DEADZONE 5    // ���ص�λ����
#define DIST_DEADZONE 0.05f // ��

#define DETECTION_THRESHOLD  2       // ���������ֵ


// ��׷�ٿ��ƺ������Ż��棩
#define MIN_DISTANCE         0.3f    // ��С��ȫ���루�ף�
#define KP_NORMAL            0.003f  // ��������ϵ��
#define KP_CLOSE             0.0015f // ���������ϵ��
#define MAX_TURN             0.8f    // ���ת���ٶȣ�rad/s��
#define D_WEIGHT             0.2f    // ΢����Ȩ��
#define FILTER_FACTOR        0.3f    // ��ͨ�˲�ϵ��


//PWM���������Сֵ
#define PWM_MAX  6900
#define PWM_MIN  -6900


#define Default_Velocity					350			//Ĭ��ң���ٶȣ���λmm/s
#define Default_Turn_Bias					Pi/4			//Ĭ��ң���ٶȣ���λ��/s
#define Bluetooth_Turn_Angle				Pi/20		//������������Ĭ��ת��Ƕȣ���λrad
#define PS2_Turn_Angle						Pi/4		//���������ֱ�Ĭ��ת��Ƕȣ���λrad

#define forward_velocity 0.25  //Move_X�ٶ�
#define Along_Angle      77    //������ķ���

#define FILTERING_TIMES 					10			//�����˲�

//С����ģʽ����
#define Normal_Mode							0
#define Lidar_Avoid_Mode					1
#define Lidar_Follow_Mode					2
#define Lidar_Along_Mode				    3		
#define ELE_Line_Patrol_Mode				4
#define CCD_Line_Patrol_Mode				5
#define ROS_Mode				            6
#define Measure_Distance_Mode				7			//Ĭ��û��ʹ��

#define Follow_Distance 1600  //�������
#define Keep_Follow_Distance 400  //���汣�־���

#define Aovid_Speed 0.3        //�����ٶ�

//ָʾң�ؿ��ƵĿ���
#define RC_ON								1	
#define RC_OFF								0
//ң�ؿ���ǰ���ٶ����ֵ
#define MAX_RC_Velocity						800
//ң�ؿ���ת���ٶ����ֵ
#define	MAX_RC_Turn_Bias					1
//ң�ؿ���ǰ���ٶ���Сֵ
#define MINI_RC_Velocity					210
//ң�ؿ���ת���ٶ���Сֵ
#define	MINI_RC_Turn_Velocity			    Pi/20

//ǰ���Ӽ��ٷ���ֵ��ÿ��ң�ؼӼ��Ĳ���ֵ
#define X_Step								25
//ת��Ӽ��ٷ���ֵ
#define Z_Step								0.1

//����ֱ��
#define Diff_Car_Wheel_diameter				0.0670f			//���ٳ��Ͱ�����������λm
#define Small_Tank_WheelDiameter			0.0430f			//С�Ĵ���
#define Big_Tank_WheelDiameter				0.0440f			//���Ĵ���

//�����־�
#define Diff_wheelspacing					0.177			//���ٳ��־�
#define Akm_wheelspacing					0.162		//���������־�
#define Small_Tank_wheelspacing				0.1350		//С�Ĵ����־�
#define Big_Tank_wheelspacing				0.2330			//���Ĵ����־�

#define Akm_axlespacing           			0.144f			//�����������
#define Diff_axlespacing                    0.155f           //���ٳ����


#define Gear_Ratio							30.0f			//����ļ��ٱ�


#define Pi									3.14159265358979f	//Բ����
#define Angle_To_Rad						57.295779513f	//�Ƕ���ת�����ƣ������������
#define Frequency							200.0f			//ÿ5ms��ȡһ�α�������ֵ
#define SERVO_INIT 							1500  			//������PWMֵ
#define SERVO3_INIT 							1900		
//������PWMֵ 500 -  550 0 600 1˿˿ 825�ܹ�ƹ����850Ҫ��ת  1100�Ƕ�
//1500�պò��� û���� 1800�Ǵ�ֱ  1900�����״����� 25

#define Encoder_resolution_Photoelectric	500.0f			//��������500��
#define Encoder_resolution_Hall 			13.0f			//����������13��
#define Encoder_resolution 					Encoder_resolution_Hall		//ʹ��13�߻���������

#define Angle_TO_PWM						640.62f			//���ڼ���pwm�ͽǶȵĹ�ϵ

#define Normal								0				//����쳣״̬��0Ϊ����
#define Abnormal							1


#define Lidar_Detect_ON						1				//���Ѳ���Ƿ����״����ϰ���
#define Lidar_Detect_OFF					0

//#define forward_velocity 250 //С����ʼ�ٶ�

//���������ݶ�ȡƵ��
#define   CONTROL_FREQUENCY 100
#define   Encoder_precision 4*Encoder_resolution_Hall*Gear_Ratio
//����ٶȿ�����ز����ṹ��
typedef struct  
{
	float Current_Encoder;     	//��������ֵ����ȡ���ʵʱ�ٶ�
	float Motor_Pwm;     		//���PWM��ֵ�����Ƶ��ʵʱ�ٶ�
	float Target_Encoder;  		//���Ŀ��������ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Velocity; 	 		//����ٶ�ֵ
}Motor_parameter;

//�������ṹ��
typedef struct  
{
  int A;      
  int B;  
}Encoder;

extern u16 mini_distance1;
extern float angle1;
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
extern float Move_X,Move_Z;
extern float PWM_Left,PWM_Right;
extern float RC_Velocity,RC_Turn_Velocity;
extern u8 Mode;
extern Motor_parameter MotorA,MotorB;	
extern int Servo_PWM;
extern u8 Lidar_Detect;
extern float CCD_Move_X,ELE_Move_X;	
extern Encoder OriginalEncoder;


//void Bluetooth_Control(void);
//void PS2_Control(void);
void Get_Target_Encoder(float Vx,float Vz);
void Get_Motor_PWM(void);
void Get_Velocity_From_Encoder(void);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

float PWM_Limit(float IN,float max,float min);
u8 Turn_Off(void);
float Mean_Filter_Left(float data);
float Mean_Filter_Right(float data);
void Lidar_Avoid(void);
void Lidar_Follow(void);
float Merge_Angle(float angle1,float angle2);
//float Get_Merge_Angle(float angle,PointDataProcessDef Data,u8 Current_Count);
void Ultrasonic_Follow(void);
void Car_Perimeter_Init(void);
void Lidar_along_wall(void);
void Get_Angle(u8 way);



void Track_Ball(void);//׷��

#endif
