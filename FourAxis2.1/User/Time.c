#include "Time.h"

void Time2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	//��ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);

	//����ʱ����0��ʼ������5��Ϊһ����ʱ����,1ms
  TIM_TimeBaseStructure.TIM_Period = 5;
  TIM_TimeBaseStructure.TIM_Prescaler = 16800-1;//һ���Ӽ���5000��,APB1Ϊ42MHz��ʱ�ӷ�Ƶ��Ϊ1������
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);
	
//	//ʹ���ж�
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );


  TIM_Cmd(TIM2, ENABLE);


  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}	

