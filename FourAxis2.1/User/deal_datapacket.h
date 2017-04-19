#ifndef DEAL_DATAPACKET_H
#define DEAL_DATAPACKET_H

#include "stm32f4xx.h"
#include "Usart.h"

void UnpackData(void);
void PackData(u8 Lock_State);
u8 CheckSum(u8* data,u8 length,u8 checkCode);
void reformat(void );
void importData(u8 res);
void errcopy(u8* source,u8* dest,u8 length);
	#endif 
