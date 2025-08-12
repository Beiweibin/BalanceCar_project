#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"                  // Device header

//�����˶�״̬-�ṹ�����
typedef struct{	
	int16_t speed[2];
	int16_t Goal_Vel;
	int16_t Vel_L;
	int16_t Vel_R;//����������ֵ
	float Car_zero;//��е����	
	float Car_pitch;//��ֱ��ƽ�� ������
	float Pitch_Acc;//������-�Ǽ��ٶ�
	float Car_Head;//��ת�� ƫ����	
	float Goal_Head;//��ת�� ƫ����	
}MotionStatus;

void MotorCtrl_StatusInit(MotionStatus *car);
extern MotionStatus CarBalc;//ȡbalance-balc

//����ƽ�⳵����pid���Ĳ���
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
float SteerPid_Output3();//ת��
void Car_Status_UpData();
void MotorCtrl_Balance();




void Motor_Init(void);
void Motor_SetSpeeds(int16_t Speeds[]);

#endif

