#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Motor.h"

#define STBY_PIN GPIO_Pin_13
#define STBY_GPIO_PORT GPIOB

void Motor_SetSpeeds(int16_t Speeds[]);

//����һ��ƽ�⳵���Ե� �ṹ��
MotionStatus CarBalc;
PidInitTypedef Stand_PID;//ֱ������������ԭ�أ�
PidInitTypedef Speed_PID;//�ٶȻ����������ٶȣ�
PidInitTypedef Steer_PID;//ת�򻷣�ƫ����ԭ�أ�

void MotorCtrl_StatusInit(MotionStatus *car) // ��ʼ��
{  
	car-> speed[0]=0;//������������PWM���ٶ�
	car-> speed[1]=0;
	car-> Goal_Vel= 0;
  car-> Vel_L = 0;//��������ˢ��
	car-> Vel_R = 0;
	
	car-> Car_zero =2.8f;//��е���ģ��ֶ�������
	car-> Car_pitch=0.0f;//�ߵ���ˢ��
	car-> Pitch_Acc=0.0f;
	
	car-> Car_Head =0.0f;
	car-> Goal_Head =0.0f;//Ŀ��ת�ĽǶ�
}

//PID�������ú���
void Set_PidValue(PidInitTypedef *PidInit, float p, float i, float d)
{
	PidInit->p = p;
	PidInit->i = i;
	PidInit->d = d;
	//�����޷�
	PidInit->maxI = 300/PidInit->p;
	//����޷�
	PidInit->maxOutput = 300;
	PidInit->minOutput = -300;
}

//��ʼ����������
void Init_All_PIDs() 
{
	Set_PidValue(&Stand_PID,26.1f, 0.0f, 0.0f);//ֱ����ֻ��Ҫpd,������Ӧ+�񵴲��������� iֵ��Ӱ�췴Ӧ�������һ���û���ȶ�ƽ���״̬�����Բ���Ҫ������̬��
	Set_PidValue(&Speed_PID,1.1f,  0.0f, 0.0f);//�ٶȻ�ֻ��pi��d���񵴻ᵼ�µ���������ٶȱ仯��������
	Set_PidValue(&Steer_PID,1.85f, 0.0f, 0.0f);//��d���Ӽ�һ��Omegaʵ�����Ҳ���
}

float StandPid_Output1()//ֱ����
{
	Stand_PID.error = CarBalc.Car_zero -CarBalc.Car_pitch;//��е����-��ǰ������
 
	/***���Ĺ�ʽ***/ 
	//ֱ��ȡ�������ٶ� d*(Stand_PID.error-Stand_PID.lastError)(��ʵ��error����Ҳ���ԣ�ԭ��һ��)
	Stand_PID.output = Stand_PID.p * Stand_PID.error+ Stand_PID.d*CarBalc.Pitch_Acc;
	
			if(Stand_PID.output<=0.5f && Stand_PID.output>0) Stand_PID.output=1;
			else if(Stand_PID.output>=-0.5f && Stand_PID.output<0) Stand_PID.output=-1;//��С���
	
//	Stand_PID.lastError =  Stand_PID.error;//΢����ֱ�����˹ߵ��Ľ��ٶȣ�����Ҫ����
	
	if(Stand_PID.output > Stand_PID.maxOutput)//����޷�
		Stand_PID.output = Stand_PID.maxOutput;	
	else if(Stand_PID.output < Stand_PID.minOutput)
		Stand_PID.output = Stand_PID.minOutput;
	
	return Stand_PID.output;
}

/*
����λ��ʽ������ʽ��ѡ�ã�Ӧ�ÿ��ǵ�����Ҫʲô���������
����������٣���Ҫ������Ǳ������죬��һ���仯�����ƣ���ϵͳӰ��С������������
��ƽ�⳵����Ҫ�����ٶȺͷ���ı仯����ϵͳӰ�������λ��
*/
float SpeedPid_Output2()//�ٶȻ�
{
	Speed_PID.lastError =(CarBalc.Vel_L +CarBalc.Vel_R) -CarBalc.Goal_Vel;//������ٶȻ���ָ�����ֵ��ٵĿ��ƣ�����ֱ��ȡ�Ͳ�/2�����㴦��
	Speed_PID.error *=0.8f;//�ɵ�ռ�ȴ���������(�൱����һ���˲�����)
	Speed_PID.error +=Speed_PID.lastError *0.2f;//����Ӧ������least����֮��lasterror��������һ�ε� Ȼ��*0.2

		Speed_PID.integral += Speed_PID.error;//�����޷�
			if(Speed_PID.integral > Speed_PID.maxI)
				Speed_PID.integral = Speed_PID.maxI;
			else if(Speed_PID.integral < -Speed_PID.maxI)  
				Speed_PID.integral = -Speed_PID.maxI;

	/***���Ĺ�ʽ***/ //
	Speed_PID.output = Speed_PID.p * Speed_PID.error + Speed_PID.i*Speed_PID.integral;
	
			if(Speed_PID.output<=0.5f && Speed_PID.output>0) Speed_PID.output=1;
			else if(Speed_PID.output>=-0.5f && Speed_PID.output<0) Speed_PID.output=-1;//��С���
			
			if(CarBalc.Car_pitch>60 || CarBalc.Car_pitch<-60) Speed_PID.integral =0;//С����������			
			
	if(Speed_PID.output > Speed_PID.maxOutput)//����޷�
		Speed_PID.output = Speed_PID.maxOutput;	
	else if(Speed_PID.output < Speed_PID.minOutput)
		Speed_PID.output = Speed_PID.minOutput;
	
	return Speed_PID.output;
}

float SteerPid_Output3()//ת��
{
	Steer_PID.error =CarBalc.Goal_Head -CarBalc.Car_Head;//������ٶȻ���ָ�����ֵ��ٵĿ��ƣ�����ֱ��ȡ�Ͳ�/2�����㴦��

	/***���Ĺ�ʽ***/ //ֱ��ȡ�������ٶ� d*(Stand_PID.error-Stand_PID.lastError)
	Steer_PID.output = Steer_PID.p * Steer_PID.error;
	
			if(Steer_PID.output<=0.5f && Steer_PID.output>0) Steer_PID.output=1;
			else if(Steer_PID.output>=-0.5f && Steer_PID.output<0) Steer_PID.output=-1;//��С���
			
	if(Steer_PID.output > Steer_PID.maxOutput)//����޷�
		Steer_PID.output = Steer_PID.maxOutput;	
	else if(Steer_PID.output < Steer_PID.minOutput)
		Steer_PID.output = Steer_PID.minOutput;
	
	return Steer_PID.output;
}

void Car_Status_UpData()
{
	//������ ǰ���򴫲θ� CarBalc.Goal_Vel =+ -50 ���ǰ�����Ҷ�û����ά��0����ֱ��
	//       �����򴫲θ� CarBalc.GoalHead =CarBalc.Car_Head+ -2��������һֱ׷�÷�����ٶȣ������������ţ���������1�ⲿ�ж� ���Զ�����
	
	//�������������µ�ǰ���٣�Vel_L��Vel_R	!!
}
void MotorCtrl_Balance()
{
	float OffVel2 =SpeedPid_Output2();
	float StandVel1 =StandPid_Output1();//����-����
	//��Ҫ����ת�ĽǶȣ��Լ�������ǰ���ź�
	float Omega3 =SteerPid_Output3();

	CarBalc.speed[0] =StandVel1 +OffVel2  -Omega3;//��
	CarBalc.speed[1] =StandVel1 +OffVel2  +Omega3;//����
	
	Motor_SetSpeeds(CarBalc.speed);
}	


void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	//GPIO_SetBits(STBY_GPIO_PORT, STBY_PIN);
	
	PWM_Init(); 
}

void Motor_SetSpeeds(int16_t Speeds[])
{
	if (Speeds[0] >= 0){
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		PWM_SetCompare2(Speeds[0]);
	}
	else{
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		PWM_SetCompare2(-Speeds[0]);
	}

	if (Speeds[1] >= 0){
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
		PWM_SetCompare1(Speeds[1]);
	}
	else{
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
		PWM_SetCompare1(-Speeds[1]);
	}
}
