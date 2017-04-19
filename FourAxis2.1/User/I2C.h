#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include "Systick.h"

//代码移植只需要更改下面4条宏定义
#define I2C_PORT GPIOB
#define I2C_SCL GPIO_Pin_6
#define I2C_SDA GPIO_Pin_7
#define RCC_I2C_PORT RCC_AHB1Periph_GPIOB
//
#define SCL_LOW (I2C_PORT->BSRRH|=I2C_SCL)
#define SCL_HIGH (I2C_PORT->BSRRL|=I2C_SCL)
#define SDA_LOW (I2C_PORT->BSRRH|=I2C_SDA)
#define SDA_HIGH (I2C_PORT->BSRRL|=I2C_SDA)
#define SDA_State (I2C_PORT->IDR&I2C_SDA)

void I2C_Config(void);
void Start(void);
void Stop(void);
void SetAck(FunctionalState ackState);
FunctionalState GetAck(void);
FunctionalState I2C_WriteByte(u8 data);
u8 I2C_ReadyByte(FunctionalState ackState);

u8 I2C_WriteByteToSlave(u8 addr,u8 reg,u8 data);
u8 I2C_WriteSomeDataToSlave(u8 addr,u8 reg,u8 len,u8 *buf); 
u8 I2C_ReadFromSlave(u8 addr,u8 reg,u8 *buf);
u8 I2C_ReadSomeDataFromSlave(u8 addr,u8 reg,u8 len,u8 *buf);


#endif
