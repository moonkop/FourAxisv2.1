#include "init.h"


u8 Init_Finish;

void WR_Init(void)
{
	//����pwm���
	Motor_Init();
	//LED���ܳ�ʼ��
	LED_Init();
	//������λ��ͨ�Ŵ���
	USART1_Config();
	//����ң�����������ݴ���
	USART2_Config();
	//���ó���������
//	USART4_Config();

	//���ö�ʱ��
	Time2_Init();
	//����i2c
	I2C_Config();
	
	//dmp��ĳ�ʼ��
	mpu_dmp_init();
	//��ѹ�Ƴ�ʼ��
//	MS5611_Init();
	
	
	//dmp�жϣ�����ʼ��
//	DMP_EXTIConfig();
	
	//��ʼ��������־
	Init_Finish=1;
	
	//ʹ���ж�
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );
}	
