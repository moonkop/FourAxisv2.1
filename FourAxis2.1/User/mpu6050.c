#include "mpu6050.h"
#include "I2C.h"
#include "Systick.h"


#define	SlaveAddress	0x68	//�ӻ��豸��ַ
/*****************************************************************************
MPU6050�ڲ��Ĵ�����ַ
*****************************************************************************/
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ0x07��125HZ��
#define	CONFIG			0x1A	//��ͨ�˲��� ������ֵ:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ:0x18(���Լ�,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ죬������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01�����Լ죬+-2g��5HZ��
#define FIFO_EN			0x23	//�����ܵ�FIFO��ʹ�ܻ�ʧ��
#define INT_PIN_CFG		0x37	//�ж�/��·����
#define INT_ENABLE		0x38	//�ж�ʧ�ܼĴ���
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
#define USER_CTRL		0x6A	//�û����ƼĴ���
#define	PWR_MGMT_1		0x6B	//��Դ����,����ֵ:0x01(????)
#define PWR_MGMT_2		0x6C	//������������ģʽ���ǻ���ģʽ
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ��ֵ0x68,ֻ��)



//��ʼ��MPU6050
void MPU6050_MoniI2c_Config(void)
{
	
	//��ʼ��MPU6050,���˯��״̬
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_1, 0x80);			//��λMPU6050
	delay_ms(100);														//??100ms
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_1, 0x00);			//����MPU6050
	
	I2C_WriteByteToSlave(SlaveAddress,GYRO_CONFIG, 0x18);			//��������������Ϊ:+-2000dps�����Լ�
	I2C_WriteByteToSlave(SlaveAddress,ACCEL_CONFIG, 0x00);			//���ü��ٶȴ���������Ϊ:+-2g
	I2C_WriteByteToSlave(SlaveAddress,SMPLRT_DIV, 0x13);			//���ò���Ƶ�ʵķ�ƵֵΪ19,�Ӷ�������Ϊ50Hz
	I2C_WriteByteToSlave(SlaveAddress,CONFIG, 0x04);				//�������ֵ�ͨ�˲�ֵ,2�˲����Ϊ20Hz
	I2C_WriteByteToSlave(SlaveAddress,USER_CTRL, 0x00);				//�ر�����FIFO,���ҹر�I2C��ģʽ
	I2C_WriteByteToSlave(SlaveAddress,FIFO_EN, 0x00);				//�رո����ܵ�FIFO
	I2C_WriteByteToSlave(SlaveAddress,INT_PIN_CFG, 0x80);			//����INT����(INT�͵�ƽ��Ч,�������...)
	I2C_WriteByteToSlave(SlaveAddress,INT_ENABLE, 0x00);			//�ر������ж�
	
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_1, 0x01);			//ʹ��X���PLLʱ����Ϊ����ʱ��
	I2C_WriteByteToSlave(SlaveAddress,PWR_MGMT_2, 0x00);			//���������Ǻͼ��ټƴ�����������ģʽ
}



//��ȡMPU6050��ID
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




