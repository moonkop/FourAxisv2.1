#ifndef LED_H
#define LED_H

#include "stm32f4xx.h"
#include "Systick.h"

#define LED4_OFF         GPIO_LED->BSRRL = Pin_LED1
#define LED4_ON          GPIO_LED->BSRRH = Pin_LED1
#define LED3_OFF         GPIO_LED->BSRRL = Pin_LED2
#define LED3_ON          GPIO_LED->BSRRH = Pin_LED2
#define LED2_OFF         GPIO_LED->BSRRL = Pin_LED3
#define LED2_ON          GPIO_LED->BSRRH = Pin_LED3
#define LED1_OFF         GPIO_LED->BSRRL = Pin_LED4
#define LED1_ON          GPIO_LED->BSRRH = Pin_LED4


#define RCC_LED			RCC_AHB1Periph_GPIOE
#define GPIO_LED		GPIOE
#define Pin_LED1		GPIO_Pin_0 //oRANGE
#define Pin_LED2		GPIO_Pin_1//GREEN
#define Pin_LED3		GPIO_Pin_2//RED
#define Pin_LED4		GPIO_Pin_3//BLUE


void LED_Init(void);
void LED_Blink(void);

void LED_Connect_Fail(void);//��ң��������ʧ��
void LED_Unlocked(void);//��ң�������ӳɹ��ҽ���
void LED_Locked(void);//��ң�������ӳɹ�������״̬
void LED_Verify_Success(void);//�յ�������ң������������У��ɹ�
void LED_Verify_Unsuccess(void);//�յ�������ң������������У�鲻�ɹ�

#endif 
