#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Motor.h"

#define STBY_PIN GPIO_Pin_13
#define STBY_GPIO_PORT GPIOB

void Motor_SetSpeeds(int16_t Speeds[]);

//声明一个平衡车属性的 结构体
MotionStatus CarBalc;
PidInitTypedef Stand_PID;//直立环（俯仰角原地）
PidInitTypedef Speed_PID;//速度环（编码器速度）
PidInitTypedef Steer_PID;//转向环（偏航角原地）

void MotorCtrl_StatusInit(MotionStatus *car) // 初始化
{  
	car-> speed[0]=0;//这个是最终输出PWM的速度
	car-> speed[1]=0;
	car-> Goal_Vel= 0;
  car-> Vel_L = 0;//编码器自刷新
	car-> Vel_R = 0;
	
	car-> Car_zero =2.8f;//机械中心（手动测量）
	car-> Car_pitch=0.0f;//惯导自刷新
	car-> Pitch_Acc=0.0f;
	
	car-> Car_Head =0.0f;
	car-> Goal_Head =0.0f;//目标转的角度
}

//PID参数设置函数
void Set_PidValue(PidInitTypedef *PidInit, float p, float i, float d)
{
	PidInit->p = p;
	PidInit->i = i;
	PidInit->d = d;
	//积分限幅
	PidInit->maxI = 300/PidInit->p;
	//输出限幅
	PidInit->maxOutput = 300;
	PidInit->minOutput = -300;
}

//初始化函数调用
void Init_All_PIDs() 
{
	Set_PidValue(&Stand_PID,26.1f, 0.0f, 0.0f);//直立环只需要pd,快速响应+振荡补偿（积分 i值会影响反应变慢，且基本没有稳定平衡的状态，所以不需要处理稳态误差）
	Set_PidValue(&Speed_PID,1.1f,  0.0f, 0.0f);//速度环只：pi（d的振荡会导致电机卡卡，速度变化不流畅）
	Set_PidValue(&Steer_PID,1.85f, 0.0f, 0.0f);//纯d叠加减一个Omega实现左右差速
}

float StandPid_Output1()//直立环
{
	Stand_PID.error = CarBalc.Car_zero -CarBalc.Car_pitch;//机械中心-当前俯仰角
 
	/***核心公式***/ 
	//直接取俯仰角速度 d*(Stand_PID.error-Stand_PID.lastError)(其实用error作差也可以，原理一样)
	Stand_PID.output = Stand_PID.p * Stand_PID.error+ Stand_PID.d*CarBalc.Pitch_Acc;
	
			if(Stand_PID.output<=0.5f && Stand_PID.output>0) Stand_PID.output=1;
			else if(Stand_PID.output>=-0.5f && Stand_PID.output<0) Stand_PID.output=-1;//最小输出
	
//	Stand_PID.lastError =  Stand_PID.error;//微分项直接用了惯导的角速度，不需要处理
	
	if(Stand_PID.output > Stand_PID.maxOutput)//输出限幅
		Stand_PID.output = Stand_PID.maxOutput;	
	else if(Stand_PID.output < Stand_PID.minOutput)
		Stand_PID.output = Stand_PID.minOutput;
	
	return Stand_PID.output;
}

/*
至于位置式和增量式的选用，应该考虑的是需要什么样的输出，
例如控制匀速，需要的输出是变慢或变快，是一个变化的趋势，对系统影响小，所以增量，
而平衡车，需要的是速度和方向的变化，对系统影响大，所以位置
*/
float SpeedPid_Output2()//速度环
{
	Speed_PID.lastError =(CarBalc.Vel_L +CarBalc.Vel_R) -CarBalc.Goal_Vel;//这里的速度环是指对两轮等速的控制，这里直接取和不/2，方便处理
	Speed_PID.error *=0.8f;//旧的占比大：流畅变速(相当于做一次滤波处理)
	Speed_PID.error +=Speed_PID.lastError *0.2f;//这里应该命名least，总之把lasterror当做最新一次的 然后*0.2

		Speed_PID.integral += Speed_PID.error;//积分限幅
			if(Speed_PID.integral > Speed_PID.maxI)
				Speed_PID.integral = Speed_PID.maxI;
			else if(Speed_PID.integral < -Speed_PID.maxI)  
				Speed_PID.integral = -Speed_PID.maxI;

	/***核心公式***/ //
	Speed_PID.output = Speed_PID.p * Speed_PID.error + Speed_PID.i*Speed_PID.integral;
	
			if(Speed_PID.output<=0.5f && Speed_PID.output>0) Speed_PID.output=1;
			else if(Speed_PID.output>=-0.5f && Speed_PID.output<0) Speed_PID.output=-1;//最小输出
			
			if(CarBalc.Car_pitch>60 || CarBalc.Car_pitch<-60) Speed_PID.integral =0;//小车倒跌清零			
			
	if(Speed_PID.output > Speed_PID.maxOutput)//输出限幅
		Speed_PID.output = Speed_PID.maxOutput;	
	else if(Speed_PID.output < Speed_PID.minOutput)
		Speed_PID.output = Speed_PID.minOutput;
	
	return Speed_PID.output;
}

float SteerPid_Output3()//转向环
{
	Steer_PID.error =CarBalc.Goal_Head -CarBalc.Car_Head;//这里的速度环是指对两轮等速的控制，这里直接取和不/2，方便处理

	/***核心公式***/ //直接取俯仰角速度 d*(Stand_PID.error-Stand_PID.lastError)
	Steer_PID.output = Steer_PID.p * Steer_PID.error;
	
			if(Steer_PID.output<=0.5f && Steer_PID.output>0) Steer_PID.output=1;
			else if(Steer_PID.output>=-0.5f && Steer_PID.output<0) Steer_PID.output=-1;//最小输出
			
	if(Steer_PID.output > Steer_PID.maxOutput)//输出限幅
		Steer_PID.output = Steer_PID.maxOutput;	
	else if(Steer_PID.output < Steer_PID.minOutput)
		Steer_PID.output = Steer_PID.minOutput;
	
	return Steer_PID.output;
}

void Car_Status_UpData()
{
	//读蓝牙 前后则传参给 CarBalc.Goal_Vel =+ -50 如果前后左右都没有则维持0，仅直立
	//       左右则传参给 CarBalc.GoalHead =CarBalc.Car_Head+ -2（长按则一直追该方向的速度）、、合理嗯嗯（蓝牙串口1外部中断 已自动处理）
	
	//读编码器，更新当前轮速：Vel_L，Vel_R	!!
}
void MotorCtrl_Balance()
{
	float OffVel2 =SpeedPid_Output2();
	float StandVel1 =StandPid_Output1();//串级-叠加
	//还要传入转的角度，以及蓝牙的前后信号
	float Omega3 =SteerPid_Output3();

	CarBalc.speed[0] =StandVel1 +OffVel2  -Omega3;//左负
	CarBalc.speed[1] =StandVel1 +OffVel2  +Omega3;//右正
	
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
