#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#include "stdio.h"

#define RX_WIDTH 12 //遥控器数据接收大小


//usart1发送给匿名上位机
void USART1_Config(void);

//遥控器接收数据使用的串口
void USART2_Config(void);

//超声波
void USART4_Config(void);


void USART1_SendByByter(u8 Data);//发送数据给上位机
void Usart1_Send(u8 *data_to_send, u8 len);//发送一个数组给上位机
void USART2_SendByByter(u8 Data);//发送数据给遥控器
void Usart2_Send(u8 *data_to_send, u8 len);//发送一个数组给遥控器
	
#endif 
