#include "Motor.h"

#define Moto_PwmMax 200

int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)//100油门为0，200油门为满
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<0)	MOTO1_PWM = 0;
	if(MOTO2_PWM<0)	MOTO2_PWM = 0;
	if(MOTO3_PWM<0)	MOTO3_PWM = 0;
	if(MOTO4_PWM<0)	MOTO4_PWM = 0;
	
	TIM1->CCR1 = MOTO4_PWM;
	TIM1->CCR2 = MOTO3_PWM;
	TIM1->CCR3 = MOTO2_PWM;
	TIM1->CCR4 = MOTO1_PWM;
}


void Motor_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_BDTRInitTypeDef TIM_BDTRInitStructure; 
		//开启时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		//配置GPIO口
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	
	
		//当定时器从0开始计数到Moto_PwmMax，为一个定时周期 ARR
		TIM_TimeBaseStruct.TIM_Period=2000;//一个pwm周期为20ms，50hz
		//设置预分期
		TIM_TimeBaseStruct.TIM_Prescaler=1680-1;//一秒钟计数100000次
		//
		TIM_TimeBaseStruct.TIM_ClockDivision = 0;	
		//向上计数模式
		TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;
		//初始化TIM_TimeBaseStruct
		TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStruct);
		
		
		//选择PWM模式
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		//允许输出模式
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
		//匹配值，初始化占空比为100，即0油门
		TIM_OCInitStruct.TIM_Pulse=100;
		//设置小于匹配值的时候输出高电平
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
		//使能定时器1的4个通道
		TIM_OC1Init(TIM1, &TIM_OCInitStruct);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能通道的CRR的预装载功能，DISABLED时会使改变CRR值时立即生效
		TIM_OC2Init(TIM1, &TIM_OCInitStruct);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM1, &TIM_OCInitStruct);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM1, &TIM_OCInitStruct);
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM的ARR的预装载功能，DISABLED时会使改变ARR值时立即生效
		//死区设置
		TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable; 
		TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable; 
		TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; 
		TIM_BDTRInitStructure.TIM_DeadTime = 0;  
		TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; 
		TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High; 
		TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; 
		TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure); 
		
		//使TIM1重载寄存器ARR
		TIM_ARRPreloadConfig(TIM1,ENABLE);
		//使能定时器1
		TIM_Cmd(TIM1,ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);//打开pwm输出
	
}
