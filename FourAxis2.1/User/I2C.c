#include "I2C.h"



void I2C_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_I2C_PORT, ENABLE);//使能时钟

  GPIO_InitStruct.GPIO_Pin = I2C_SCL|I2C_SDA;	    
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOB, &GPIO_InitStruct);	 
	
	SCL_LOW;
	delay_us(1);
	SDA_HIGH;
	delay_us(1);
	
}

//内部函数，判断总线是否处于空闲状态，若不是，则循环的提供SCL驱动，直到从机释放SDA线
void _I2C_IsBusy(void)
{
	//读取SDA状态，如果低电平，说明总线被从机控制，若为高电平说明总线空闲	，可以准备发送开始条件
	while(!SDA_State){
		SCL_LOW;
		delay_us(3);
		SCL_HIGH;
		delay_us(3);
	}
}	



//起始信号
void Start(void)
{
	//判断总线是否空闲
	_I2C_IsBusy();
		
	//先拉低SCL，防止因为SCL线在高电平而使后面SDA拉高时，产生一个STOP信号
	SCL_LOW;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
	
	SCL_HIGH;
	delay_us(1);
	
	SDA_LOW;
	delay_us(1);
	
	//把SCL拉低，钳住SCL线，准备发送地址数据
	SCL_LOW;
	delay_us(1);
}

//停止信号
void Stop(void)
{
	SCL_LOW;
	delay_us(1);
	
	SDA_LOW;
	delay_us(1);
	
	SCL_HIGH;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
}

//主机发出应答或者非应答信号
void SetAck(FunctionalState ackState)
{
	SCL_LOW;
	delay_us(1);
	
	if(ackState==ENABLE)
	{
		SDA_LOW;//应答信号
		delay_us(1);
	}
	else
	{
		SDA_HIGH;//非应答信号
		delay_us(1);
	}
	
	SCL_HIGH;
	delay_us(2);
	
	//释放主机对SDA的控制权
	SCL_LOW;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
}

//获取应答或非应答信号
FunctionalState GetAck(void)
{
	FunctionalState ackState;
	
	SCL_HIGH;
	delay_us(1);
	
	if(SDA_State)
		ackState=DISABLE;
	else
		ackState=ENABLE;
	
	//拉低，取走数据
	SCL_LOW;
	delay_us(1);
	
	return ackState;
}

//写数据给从机，并返回应答或者非应答信号
FunctionalState I2C_WriteByte(u8 data)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		SCL_LOW;
		delay_us(1);
		
		if(data&0x80)
		{
			SDA_HIGH;
			delay_us(1);
		}
		else
		{
			SDA_LOW;
			delay_us(1);
		}
		
		data<<=1;
		delay_us(1);
		
		SCL_HIGH;
		delay_us(2);
		
	}
	
	//主机释放SDA线，使得总线空闲，以便mpu6050能发出响应信息，并钳住SCL线
	SCL_LOW;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
	
	
	return GetAck();
}

//从从机获取数据，并决定发送应答或非应答信号
u8 I2C_ReadyByte(FunctionalState ackState)
{
	u8 i;
	u8 data=0x00;
	
	for(i=0;i<8;i++)
	{
		SCL_HIGH;
		delay_us(1);
		
		data<<=1;
		
		if(SDA_State)
		{
			data|=0x01;
		}
		
		SCL_LOW;
		delay_us(2);
	}	
	//发送应答或非应答信号
	SetAck(ackState);
		
	return data;
}

/*************************
往从机的寄存器写入一个数据
addr  设备地址
reg   寄存器地址
data  要写入的数据
返回值为0则成功，否则则为失败
**************************/
u8 I2C_WriteByteToSlave(u8 addr,u8 reg,u8 data)
{
	FunctionalState state;
	Start();//产生起始信号
	state=I2C_WriteByte(addr<<1|0);//将从机地址和写方向写入总线
	if(state==ENABLE)//判断是否响应了地址
	{
		state=I2C_WriteByte(reg);//将要写入的寄存器地址
		if(state==ENABLE)
		{
			state=I2C_WriteByte(data);//将数据发送给从机
			
			//正常写入，产生停止信号，并返回0
			Stop();
			return 0;
		}
	}
	//写入异常，产生停止信号，并返回0
  Stop();
	return 1;
}


/************************************************************
往从机的寄存器写入多个数据
addr  设备地址
reg   寄存器地址
len   要写入数据的个数
buf   要写入数据的首地址 
**************************************************************/
u8 I2C_WriteSomeDataToSlave(u8 addr,u8 reg,u8 len,u8 *buf)
{
	FunctionalState state;
	u8 i;
	Start();//产生起始信号
	state=I2C_WriteByte(addr<<1|0);//将从机地址和写方向写入总线
	if(state==ENABLE)//判断是否响应了地址
	{
		state=I2C_WriteByte(reg);//将要写入的寄存器地址
		if(state==ENABLE)
		{			
			for(i=0;i<len;i++)
			{
				state=I2C_WriteByte(*(buf+i));//将数据发送给从机
				if(state == DISABLE)//从机未应答，停止传输数据并产生停止信号且返回1
				{					
					Stop();                        
					return 1;
				}
			}
			//正常写入，产生停止信号，并返回0
			Stop();
			return 0;
		}
	}
	//写入异常，产生停止信号，并返回0
  Stop();
	return 1;
}



/***********************************************
往从机的寄存器读取一个字节的数据
addr  设备地址
reg   寄存器地址
buf   读取到的数据存储的内存区
***********************************************/
u8 I2C_ReadFromSlave(u8 addr,u8 reg,u8 *buf)
{
	FunctionalState state;
	Start();//产生起始信号
	state=I2C_WriteByte(addr<<1|0);//将从机地址和写方向写入总线
	if(state==ENABLE)
	{
		state=I2C_WriteByte(reg);//将要写入的寄存器地址
		if(state==ENABLE)
		{
			Start();//产生起始信号
			state=I2C_WriteByte(addr<<1|1);//将从机地址和读方向写入总线
			if(state == ENABLE)
			{
				*buf = I2C_ReadyByte(DISABLE);//读取完一个数据后，向从机发送一个NACK信号，结束从机发送数据
				//正常读取数据，产生停止信号并返回0
				Stop();
				return 0;
			}
		}
	}
	//读取异常，产生停止信号，并返回0
  Stop();
	return 1;
}



/***********************************************
往从机的寄存器读取一个字节的数据
addr  设备地址
reg   寄存器地址
len   读取的数据个数 
buf   读取到的数据存储的内存区
***********************************************/
u8 I2C_ReadSomeDataFromSlave(u8 addr,u8 reg,u8 len,u8 *buf)
{
	FunctionalState state;
	Start();//产生起始信号
	state=I2C_WriteByte(addr<<1|0);//将从机地址和写方向写入总线
	if(state==ENABLE)
	{
		state=I2C_WriteByte(reg);//将要写入的寄存器地址
		if(state==ENABLE)
		{
			Start();//产生起始信号
			state=I2C_WriteByte(addr<<1|1);//将从机地址和读方向写入总线
			if(state == ENABLE)
			{
				while(len)
				{
					if(len!=1)
						*buf = I2C_ReadyByte(ENABLE);//读取完一个数据后，向从机发送一个ACK信号，从机继续发送数据
					else
						*buf = I2C_ReadyByte(DISABLE);//读取完一个数据后，向从机发送一个NACK信号，结束从机发送数据
					buf++;
					len--;
				}
				//正常读取数据，产生停止信号并返回0
				Stop();
				return 0;
			}
		}
	}
	//读取异常，产生停止信号，并返回0
  Stop();
	return 1;
}


