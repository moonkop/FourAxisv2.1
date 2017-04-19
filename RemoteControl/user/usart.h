#ifndef _USART_H
#define _USART_H

#include "stm32f10x.h"
#include "stdio.h"

void USART_Config(void);
void Usart1_Send(u8 *data_to_send, u8 len);
void USART1_SendByByter(u8 Data);

#endif
