#include "I2C.h"



void I2C_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_I2C_PORT, ENABLE);//ʹ��ʱ��

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

//�ڲ��������ж������Ƿ��ڿ���״̬�������ǣ���ѭ�����ṩSCL������ֱ���ӻ��ͷ�SDA��
void _I2C_IsBusy(void)
{
	//��ȡSDA״̬������͵�ƽ��˵�����߱��ӻ����ƣ���Ϊ�ߵ�ƽ˵�����߿���	������׼�����Ϳ�ʼ����
	while(!SDA_State){
		SCL_LOW;
		delay_us(3);
		SCL_HIGH;
		delay_us(3);
	}
}	



//��ʼ�ź�
void Start(void)
{
	//�ж������Ƿ����
	_I2C_IsBusy();
		
	//������SCL����ֹ��ΪSCL���ڸߵ�ƽ��ʹ����SDA����ʱ������һ��STOP�ź�
	SCL_LOW;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
	
	SCL_HIGH;
	delay_us(1);
	
	SDA_LOW;
	delay_us(1);
	
	//��SCL���ͣ�ǯסSCL�ߣ�׼�����͵�ַ����
	SCL_LOW;
	delay_us(1);
}

//ֹͣ�ź�
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

//��������Ӧ����߷�Ӧ���ź�
void SetAck(FunctionalState ackState)
{
	SCL_LOW;
	delay_us(1);
	
	if(ackState==ENABLE)
	{
		SDA_LOW;//Ӧ���ź�
		delay_us(1);
	}
	else
	{
		SDA_HIGH;//��Ӧ���ź�
		delay_us(1);
	}
	
	SCL_HIGH;
	delay_us(2);
	
	//�ͷ�������SDA�Ŀ���Ȩ
	SCL_LOW;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
}

//��ȡӦ����Ӧ���ź�
FunctionalState GetAck(void)
{
	FunctionalState ackState;
	
	SCL_HIGH;
	delay_us(1);
	
	if(SDA_State)
		ackState=DISABLE;
	else
		ackState=ENABLE;
	
	//���ͣ�ȡ������
	SCL_LOW;
	delay_us(1);
	
	return ackState;
}

//д���ݸ��ӻ���������Ӧ����߷�Ӧ���ź�
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
	
	//�����ͷ�SDA�ߣ�ʹ�����߿��У��Ա�mpu6050�ܷ�����Ӧ��Ϣ����ǯסSCL��
	SCL_LOW;
	delay_us(1);
	
	SDA_HIGH;
	delay_us(1);
	
	
	return GetAck();
}

//�Ӵӻ���ȡ���ݣ�����������Ӧ����Ӧ���ź�
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
	//����Ӧ����Ӧ���ź�
	SetAck(ackState);
		
	return data;
}

/*************************
���ӻ��ļĴ���д��һ������
addr  �豸��ַ
reg   �Ĵ�����ַ
data  Ҫд�������
����ֵΪ0��ɹ���������Ϊʧ��
**************************/
u8 I2C_WriteByteToSlave(u8 addr,u8 reg,u8 data)
{
	FunctionalState state;
	Start();//������ʼ�ź�
	state=I2C_WriteByte(addr<<1|0);//���ӻ���ַ��д����д������
	if(state==ENABLE)//�ж��Ƿ���Ӧ�˵�ַ
	{
		state=I2C_WriteByte(reg);//��Ҫд��ļĴ�����ַ
		if(state==ENABLE)
		{
			state=I2C_WriteByte(data);//�����ݷ��͸��ӻ�
			
			//����д�룬����ֹͣ�źţ�������0
			Stop();
			return 0;
		}
	}
	//д���쳣������ֹͣ�źţ�������0
  Stop();
	return 1;
}


/************************************************************
���ӻ��ļĴ���д��������
addr  �豸��ַ
reg   �Ĵ�����ַ
len   Ҫд�����ݵĸ���
buf   Ҫд�����ݵ��׵�ַ 
**************************************************************/
u8 I2C_WriteSomeDataToSlave(u8 addr,u8 reg,u8 len,u8 *buf)
{
	FunctionalState state;
	u8 i;
	Start();//������ʼ�ź�
	state=I2C_WriteByte(addr<<1|0);//���ӻ���ַ��д����д������
	if(state==ENABLE)//�ж��Ƿ���Ӧ�˵�ַ
	{
		state=I2C_WriteByte(reg);//��Ҫд��ļĴ�����ַ
		if(state==ENABLE)
		{			
			for(i=0;i<len;i++)
			{
				state=I2C_WriteByte(*(buf+i));//�����ݷ��͸��ӻ�
				if(state == DISABLE)//�ӻ�δӦ��ֹͣ�������ݲ�����ֹͣ�ź��ҷ���1
				{					
					Stop();                        
					return 1;
				}
			}
			//����д�룬����ֹͣ�źţ�������0
			Stop();
			return 0;
		}
	}
	//д���쳣������ֹͣ�źţ�������0
  Stop();
	return 1;
}



/***********************************************
���ӻ��ļĴ�����ȡһ���ֽڵ�����
addr  �豸��ַ
reg   �Ĵ�����ַ
buf   ��ȡ�������ݴ洢���ڴ���
***********************************************/
u8 I2C_ReadFromSlave(u8 addr,u8 reg,u8 *buf)
{
	FunctionalState state;
	Start();//������ʼ�ź�
	state=I2C_WriteByte(addr<<1|0);//���ӻ���ַ��д����д������
	if(state==ENABLE)
	{
		state=I2C_WriteByte(reg);//��Ҫд��ļĴ�����ַ
		if(state==ENABLE)
		{
			Start();//������ʼ�ź�
			state=I2C_WriteByte(addr<<1|1);//���ӻ���ַ�Ͷ�����д������
			if(state == ENABLE)
			{
				*buf = I2C_ReadyByte(DISABLE);//��ȡ��һ�����ݺ���ӻ�����һ��NACK�źţ������ӻ���������
				//������ȡ���ݣ�����ֹͣ�źŲ�����0
				Stop();
				return 0;
			}
		}
	}
	//��ȡ�쳣������ֹͣ�źţ�������0
  Stop();
	return 1;
}



/***********************************************
���ӻ��ļĴ�����ȡһ���ֽڵ�����
addr  �豸��ַ
reg   �Ĵ�����ַ
len   ��ȡ�����ݸ��� 
buf   ��ȡ�������ݴ洢���ڴ���
***********************************************/
u8 I2C_ReadSomeDataFromSlave(u8 addr,u8 reg,u8 len,u8 *buf)
{
	FunctionalState state;
	Start();//������ʼ�ź�
	state=I2C_WriteByte(addr<<1|0);//���ӻ���ַ��д����д������
	if(state==ENABLE)
	{
		state=I2C_WriteByte(reg);//��Ҫд��ļĴ�����ַ
		if(state==ENABLE)
		{
			Start();//������ʼ�ź�
			state=I2C_WriteByte(addr<<1|1);//���ӻ���ַ�Ͷ�����д������
			if(state == ENABLE)
			{
				while(len)
				{
					if(len!=1)
						*buf = I2C_ReadyByte(ENABLE);//��ȡ��һ�����ݺ���ӻ�����һ��ACK�źţ��ӻ�������������
					else
						*buf = I2C_ReadyByte(DISABLE);//��ȡ��һ�����ݺ���ӻ�����һ��NACK�źţ������ӻ���������
					buf++;
					len--;
				}
				//������ȡ���ݣ�����ֹͣ�źŲ�����0
				Stop();
				return 0;
			}
		}
	}
	//��ȡ�쳣������ֹͣ�źţ�������0
  Stop();
	return 1;
}


