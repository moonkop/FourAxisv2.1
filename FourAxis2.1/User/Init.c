#include "init.h"


u8 Init_Finish;

void WR_Init(void)
{
	//配置pwm输出
	Motor_Init();
	//LED功能初始化
	LED_Init();
	//配置上位机通信串口
	USART1_Config();
	//配置遥控器接收数据串口
	USART2_Config();
	//配置超声波串口
//	USART4_Config();

	//配置定时器
	Time2_Init();
	//配置i2c
	I2C_Config();
	
	//dmp库的初始化
	mpu_dmp_init();
	//气压计初始化
//	MS5611_Init();
	
	
	//dmp中断，最后初始化
//	DMP_EXTIConfig();
	
	//初始化结束标志
	Init_Finish=1;
	
	//使能中断
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );
}	
