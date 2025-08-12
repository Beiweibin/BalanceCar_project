#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Gyro.h"
#include "timer.h"
#include "Bluetooth.h"
#include "serial3.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"

uint16_t TIM_CNT_1MS = 0;
uint16_t TIM_CNT_5MS = 0;
uint16_t TIM_CNT_20MS = 0;
uint32_t TIM_CNT_TotalMS = 0;

void Loop_Init_TIM_CNT(void)
{
	TIM_CNT_1MS = 0;
	TIM_CNT_5MS = 0;
	TIM_CNT_20MS = 0;
}


void Time_Second_1ms_Serve(void)
{
}
void Time_Second_5ms_Serve(void)//主要执行
{
	Car_Status_UpData();
	MotorCtrl_Balance();
	
}
void Time_Second_20ms_Serve(void)
{		

//  Serial_Printf("%f,%f,%f\r\n",out,target_angle,fRoll);//输出out,目标角度--实际角度
}


void Time_Second_Run()
{
	static uint32_t TIM_CNT_1MS_OLD = 0;
	
	if(TIM_CNT_1MS_OLD != TIM_CNT_1MS)		{Time_Second_1ms_Serve();		}
	if(TIM_CNT_5MS)		  {TIM_CNT_5MS--;		Time_Second_5ms_Serve();		}
	if(TIM_CNT_20MS)		{TIM_CNT_20MS--;		Time_Second_20ms_Serve();	}
	
	TIM_CNT_1MS_OLD = TIM_CNT_1MS;
}


void Loop_1ms_BaseTimer(void)	//记录当前的秒数
{
	TIM_CNT_1MS++;
	TIM_CNT_TotalMS++;
	if(!(TIM_CNT_1MS % 5))	{TIM_CNT_5MS++;}
	if(!(TIM_CNT_1MS % 20))	{TIM_CNT_20MS++;}
}
void TIM2_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_InitStruct.TIM_Prescaler = 7200-1;    // 72MHz/7200=10kHz
    TIM_InitStruct.TIM_Period = 150-1;        // 100Hz (10ms)
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM_Cmd(TIM2, ENABLE);
}
void TIM2_IRQHandler(void) {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
			Loop_1ms_BaseTimer();//一直计时1ms++;
    }
}
