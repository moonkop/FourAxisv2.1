#ifndef MS5611_H
#define MS5611_H

#include "stm32f4xx.h"

void MS5611_BaroAltCalculate(void);
int MS5611_Update(void);
void MS5611_Init(void);


void MS5611_Read_measure(u8 * rxbuf);
void MS5611_Start_T(void);
void MS5611_Start_P(void);
void MS5611_Reset(void);
u8 MS5611_Read_Prom(uint16_t * ms5611_prom8);
#endif 
