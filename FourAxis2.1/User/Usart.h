#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#include "stdio.h"

#define RX_WIDTH 12 //ң�������ݽ��մ�С


//usart1���͸�������λ��
void USART1_Config(void);

//ң������������ʹ�õĴ���
void USART2_Config(void);

//������
void USART4_Config(void);


void USART1_SendByByter(u8 Data);//�������ݸ���λ��
void Usart1_Send(u8 *data_to_send, u8 len);//����һ���������λ��
void USART2_SendByByter(u8 Data);//�������ݸ�ң����
void Usart2_Send(u8 *data_to_send, u8 len);//����һ�������ң����
	
#endif 
