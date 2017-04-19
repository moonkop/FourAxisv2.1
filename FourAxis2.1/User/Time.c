#include "Time.h"

void Time2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	//打开时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);

	//当定时器从0开始计数到5，为一个定时周期,1ms
  TIM_TimeBaseStructure.TIM_Period = 5;
  TIM_TimeBaseStructure.TIM_Prescaler = 16800-1;//一秒钟计数5000次,APB1为42MHz，时钟分频不为1，翻倍
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);
	
//	//使能中断
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );


  TIM_Cmd(TIM2, ENABLE);


  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}	

