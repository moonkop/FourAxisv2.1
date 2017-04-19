#ifndef INIT_H
#define INIT_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "Usart.h"
#include "Systick.h"
#include "LED.h"
#include "Motor.h"
#include "Time.h"
#include "I2C.h"
#include "dmp_exti.h"
#include "mpu_dmp_api.h"
#include "ms5611.h"


//初始化结束标志
extern u8 Init_Finish;

void WR_Init(void);

#endif 
