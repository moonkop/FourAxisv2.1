#include "Init.h"

#include "mpu6050.h"
#include "mpu_dmp_api.h"

#include "scheduler.h"

#include"DEBUG.h"
int main(void)
{	

  WR_Init();

	//USART1_Config();
	//USART2_Config();
//	USART4_Config();

	while(1)
	{
		 //delay_ms(1);
		//printf("%d",cnt);
//		LED_Blink();
		//USART1_SendByByter(a);
	//  printf("fuck!!!\n");
		//delay_ms(700);
		Main_Loop();

	}
} 

