#ifndef CONTROL_H
#define CONTROL_H

#include "stm32f4xx.h"






void Get_Angle(void);
void CTRL_angular_velocity(void);
void CTRL_attitude(void);
void Direction_Control(void);
void DealPwm(void);
void Set_Pwm(void);

void Control_test(void);

#endif
