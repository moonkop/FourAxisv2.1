#include "mpu6050.h"
#include "I2C.h"
#include "Systick.h"


#define	SlaveAddress	0x68	//从机设备地址
/*****************************************************************************
MPU6050内部寄存器地址
*****************************************************************************/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率：典型值0x07（125HZ）
#define	CONFIG			0x1A	//低通滤波器 ，典型值:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值:0x18(不自检,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检，测量范围，高通滤波频率，典型值：0x01（不自检，+-2g，5HZ）
#define FIFO_EN			0x23	//各功能的FIFO的使能或失能
#define INT_PIN_CFG		0x37	//中断/旁路设置
#define INT_ENABLE		0x38	//中断失能寄存器
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define USER_CTRL		0x6A	//用户控制寄存器
#define	PWR_MGMT_1		0x6B	//电源管理,典型值:0x01(????)
#define PWR_MGMT_2		0x6C	//设置正常工作模式还是唤醒模式
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认值0x68,只读)



//初始化MPU6050
void MPU6050_MoniI2c_Config(void)
{
	
	//初始化MPU6050,解除睡眠状态
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_1, 0x80);			//复位MPU6050
	delay_ms(100);														//??100ms
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_1, 0x00);			//唤醒MPU6050
	
	I2C_WriteByteToSlave(SlaveAddress,GYRO_CONFIG, 0x18);			//设置陀螺仪量程为:+-2000dps，不自检
	I2C_WriteByteToSlave(SlaveAddress,ACCEL_CONFIG, 0x00);			//设置加速度传感器量程为:+-2g
	I2C_WriteByteToSlave(SlaveAddress,SMPLRT_DIV, 0x13);			//设置采样频率的分频值为19,从而采样率为50Hz
	I2C_WriteByteToSlave(SlaveAddress,CONFIG, 0x04);				//设置数字低通滤波值,2滤波宽带为20Hz
	I2C_WriteByteToSlave(SlaveAddress,USER_CTRL, 0x00);				//关闭所有FIFO,并且关闭I2C主模式
	I2C_WriteByteToSlave(SlaveAddress,FIFO_EN, 0x00);				//关闭各功能的FIFO
	I2C_WriteByteToSlave(SlaveAddress,INT_PIN_CFG, 0x80);			//设置INT引脚(INT低电平有效,推挽输出...)
	I2C_WriteByteToSlave(SlaveAddress,INT_ENABLE, 0x00);			//关闭所有中断
	
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_1, 0x01);			//使用X轴的PLL时钟作为工作时钟
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_2, 0x00);			//设置陀螺仪和加速计处于正常工作模式
}



//读取MPU6050的ID
u8 MPU6050_MoniI2c_GetId(void)
{	
	u8 data;
	if(!I2C_ReadFromSlave(SlaveAddress,WHO_AM_I,&data)){
		return data;
	}else{
		return 0;
	}
}



u16 MPU6050_MoniI2c_GetAccX(void)
{
	u8 tem[2];
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,ACCEL_XOUT_H,2,tem)){
		return tem[0]<<8|tem[1];
	}else{
		return 0;
	}
}


u16 MPU6050_MoniI2c_GetAccY(void)
{
	u8 tem[2];
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,ACCEL_YOUT_H,2,tem)){
		return tem[0]<<8|tem[1];
	}else{
		return 0;
	}
}


u16 MPU6050_MoniI2c_GetAccZ(void)
{
	u8 tem[2];
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,ACCEL_ZOUT_H,2,tem)){
		return tem[0]<<8|tem[1];
	}else{
		return 0;
	}
}



u16 MPU6050_MoniI2c_GetGyroX(void)
{
	u8 tem[2];
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,GYRO_XOUT_H,2,tem)){
		return tem[0]<<8|tem[1];
	}else{
		return 0;
	}
}


u16 MPU6050_MoniI2c_GetGyroY(void)
{
	u8 tem[2];
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,GYRO_YOUT_H,2,tem)){
		return tem[0]<<8|tem[1];
	}else{
		return 0;
	}
}


u16 MPU6050_MoniI2c_GetGyroZ(void)
{
	u8 tem[2];
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,GYRO_ZOUT_H,2,tem)){
		return tem[0]<<8|tem[1];
	}else{
		return 0;
	}
}



float MPU6050_MoniI2c_GetTemperature(void)
{
	u8 tem[2];
	short value;		
	
	if(!I2C_ReadSomeDataFromSlave(SlaveAddress,TEMP_OUT_H,2,tem)){
		value = (short)(tem[0]<<8|tem[1]);
		return (36.53+(value/ 340.0));
	}else{
		return 0;
	}
}




