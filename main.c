#include "stm32f10x.h" 
#include "Motor.h"
#include "Gyro.h"
#include "timer.h"
#include "Bluetooth.h"
#include "serial3.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"

int16_t a=0;

/* main.c */
int main(void)
{
	TIM2_Init(); //��ʱ 
	Bluetooth_Init();//����ģ��u1
	USART2_Init();//��̬������u2   
	USART3_Init();//��ӡ����u3
	
	OLED_Init();
	LED_Init();
	Key_Init();  
	
	Init_All_PIDs();
	Motor_Init();
	
		for(;;)
		{		
			Time_Second_Run();
		}
	
  while(1){}
}
