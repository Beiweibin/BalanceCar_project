#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"                  // Device header

//车子运动状态-结构体参数
typedef struct{	
	int16_t speed[2];
	int16_t Goal_Vel;
	int16_t Vel_L;
	int16_t Vel_R;//即编码器传值
	float Car_zero;//机械中心	
	float Car_pitch;//即直立平衡 俯仰角
	float Pitch_Acc;//俯仰角-角加速度
	float Car_Head;//即转向 偏航角	
	float Goal_Head;//即转向 偏航角	
}MotionStatus;

void MotorCtrl_StatusInit(MotionStatus *car);
extern MotionStatus CarBalc;//取balance-balc

//设置平衡车三个pid环的参数
typedef struct
{
	float p;
	float i;
	float d;
	float goal;
	
	float current_x;
	float current_y;
	
	float error;
	float lastError;
	float integral;
	float maxI;
	float output;
	float maxOutput;
	float minOutput;
}PidInitTypedef;

void Set_PidValue(PidInitTypedef *PidInit, float p, float i, float d);
void Init_All_PIDs();

extern PidInitTypedef Stand_PID;
extern PidInitTypedef Speed_PID;
extern PidInitTypedef Turn_PID;


float StandPid_Output1();
float SpeedPid_Output2();
float SteerPid_Output3();//转向
void Car_Status_UpData();
void MotorCtrl_Balance();




void Motor_Init(void);
void Motor_SetSpeeds(int16_t Speeds[]);

#endif

