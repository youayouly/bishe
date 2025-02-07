#ifndef __PID_H
#define	__PID_H

#include "stm32f10x.h"
#include "Header.h"
extern float Velocity_KP,Velocity_KI;					//����ʽPI����
extern float Distance_KP,Distance_KD,Distance_KI;		//�������PID����
extern float Follow_KP,Follow_KD,Follow_KI;				//����ģʽת���PID���Ʋ���
extern float Follow_KP_Akm,Follow_KD_Akm,Follow_KI_Akm;
extern float Follow_KP_Diff ,Follow_KD_Diff ,Follow_KI_Diff ;	


//extern float ELE_KP_Akm,ELE_KD_Akm,ELE_KI_Akm,ELE_K_Akm;

//extern float ELE_KP_Diff,ELE_KD_Diff,ELE_KI_Diff,ELE_K_Diff;

//extern float ELE_KP_STank,ELE_KD_STank,ELE_KI_STank,ELE_K_STank;

//extern float ELE_KP_BTank,ELE_KD_BTank,ELE_KI_BTank,ELE_K_BTank;

//extern float CCD_KP_Akm,CCD_KD_Akm,CCD_KI_Akm;

//extern float CCD_KP_Diff,CCD_KD_Diff,CCD_KI_Diff;

//extern float CCD_KP_STank,CCD_KD_STank,CCD_KI_STank;

//extern float CCD_KP_BTank,CCD_KD_BTank,CCD_KI_BTank;

//extern float Diff_Along_Distance_KP,Diff_Along_Distance_KD,Diff_Along_Distance_KI;

extern float Akm_Along_Distance_KP,Akm_Along_Distance_KD,Akm_Along_Distance_KI;

//extern float STank_Along_Distance_KP,STank_Along_Distance_KD,STank_Along_Distance_KI;

//extern float BTank_Along_Distance_KP,BTank_Along_Distance_KD,BTank_Along_Distance_KI;


int Incremental_PI_Left(float Encoder,float Target);//����pid����
int Incremental_PI_Right(float Encoder,float Target);//�ҵ��pid����
float Follow_Turn_PID(float Current_Angle,float Target_Angle);
float Distance_Adjust_PID(float Current_Distance,float Target_Distance);
//float ELE_PID(int Current_ELE_ADC,int Target_ELE_ADC );

//float CCD_PID(float Current_Value,float Target_Value );
float Along_Adjust_PID(float Current_Distance,float Target_Distance);

void control_motor_steering(uint8_t type, int16_t x, int16_t y);//���pid

void Reset_Incremental_PI_Left(void);
void Reset_Incremental_PI_Right(void);
#endif
