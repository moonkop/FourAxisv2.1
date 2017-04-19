#include "LED.h"

void LED_Init()
{
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_LED,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Pin   = Pin_LED1| Pin_LED2| Pin_LED3| Pin_LED4;
    GPIO_Init(GPIO_LED, &GPIO_InitStructure);

    GPIO_SetBits(GPIO_LED, Pin_LED1);
    GPIO_SetBits(GPIO_LED, Pin_LED2);
    GPIO_SetBits(GPIO_LED, Pin_LED3);
    GPIO_SetBits(GPIO_LED, Pin_LED4);
	
}


void LED_Blink(void)
{
		LED1_ON;		
		LED2_OFF;		
		LED3_OFF;
		LED4_OFF;
		delay_ms(500);
		
		LED1_OFF;		
		LED2_ON;		
		LED3_OFF;
		LED4_OFF;
		delay_ms(500);
		
		LED1_OFF;		
		LED2_OFF;		
		LED3_ON;
		LED4_OFF;
		delay_ms(500);
		
		LED1_OFF;		
		LED2_OFF;		
		LED3_OFF;
		LED4_ON;
		delay_ms(500);
}


void LED_Connect_Fail(void)//��ң��������ʧ��
{
	LED3_ON;               		   
	LED2_OFF;
	LED1_OFF;
}
void LED_Unlocked(void)//��ң�������ӳɹ��ҽ���
{
	LED3_OFF;			
	LED2_ON;
	LED1_OFF;
}
void LED_Locked(void)//��ң�������ӳɹ�������״̬
{
	LED3_OFF;			      	
	LED2_OFF;
	LED1_ON;
}

void LED_Verify_Success(void) //�յ�������ң������������У��ɹ�
{
	LED4_ON;
	LED3_OFF;
}

void LED_Verify_Unsuccess(void) //�յ�������ң������������У�鲻�ɹ�
{
	LED3_ON;
	LED4_OFF;
}

