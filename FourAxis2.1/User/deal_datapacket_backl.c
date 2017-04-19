#include "deal_datapacket.h"
#include "led.h"
#include "DEBUG.h"
u8 dataPID;												//保存接收到的数据包识别PID值
u8 buttonFlag = 0x00;									//指示哪个键被按下，0x00表示4个按键都没有被触发
vu16 remoteControl[4];									//将从遥控器中接收到的数据重新拼装

u8 packetData[10];            //打包后待发给遥控器的数据
u8 send_dataPID = 0;	        //发送的数据包识别PID值

extern u8 rxbuf[12];									//数据接收缓存区
extern float pitch,roll,yaw;          //姿态角
//===========================================================================================


/******************************************************************************************
解包接收到的遥控器数据，并根据发送过来的数据进行相应的处理

按照下面的通讯协议进行解码，以字节为单位
前导码-按键MASK--ADC1低8--ADC1高8--ADC2低8--ADC2高8--ADC3低8--ADC3高8--ADC4低8--ADC4高8--数据包表识--校验码0xa5
其中:前导码只有0x01和0x08才表示有效的数据包,0x01表示此数据包是由ADC采样完成触发,0x08表示是由遥控器按键触发的
数据报标识用于是否是同一数据包的作用(主要用于当遥控信号中断时，飞机开始自动降落)
******************************************************************************************/
s32 err_cnt=0;
u8 errbufs[30][12];

void errcopy(u8* source,u8* dest,u8 length)
{
	while(length--)
			*(dest++)=*(source++);
}
void UnpackData(void)
{
	u8 sum=0;
	for(int i =0;i<11;i++)
	{
		sum+=rxbuf[i];
	}
	if (sum!=rxbuf[11])
	{
		errcopy(rxbuf, errbufs[err_cnt % 30], 12);
		err_cnt++;
		LED3_ON;
		LED4_OFF;
		//SEND(rxbuf,11);
		return;
	}
	LED4_ON;
	LED3_OFF;
	if(rxbuf[0] & 0x01){								//当数据包是由遥控器的ADC采样完成时触发发送时
		remoteControl[0] = rxbuf[3]<<8|rxbuf[2];		//ADC2
		remoteControl[1] = rxbuf[5]<<8|rxbuf[4];		//ADC1
		remoteControl[2] = rxbuf[7]<<8|rxbuf[6];		//ADC4
		remoteControl[3] = rxbuf[9]<<8|rxbuf[8];		//ADC3
	}else if(rxbuf[0] & 0x08){							//当数据包是由遥控器的按键触发发送时
		buttonFlag = rxbuf[1];
	}
	
	//将数据包识别PID值取出，覆盖之前的值，以表示信号链接正常
	dataPID = rxbuf[10];
}


/****************************************************************************
打包即将发给遥控器的数据

按照下面的通讯协议进行打包：(姿态需要转换u32位)
前导码--解锁码--pitch低8位--pitch高8位--roll低8位--roll高8位--yaw低8位--yaw高8位--数据包标识--校验码
其中：前导码只有0xAA才表示有效的数据包

****************************************************************************/
void PackData(u8 Lock_State)
{
	
	if(send_dataPID>=200){
		send_dataPID = 0;
	}else{
		send_dataPID++;
	}
	
	//===直接采用指针操作内存中的数值将16位转成8位，速度快，且不会发生精度截取的现象，注意：stm32是小端地址
	packetData[0]  = 0xAA;
	packetData[1]  = Lock_State;
  packetData[2]  = *((u8*)&pitch);
  packetData[3]  = *(((u8*)&pitch)+1);
  packetData[4]  = *((u8*)&roll);
  packetData[5]  = *(((u8*)&roll)+1);
  packetData[6]  = *((u8*)&yaw);
  packetData[7]  = *(((u8*)&yaw)+1);
  packetData[8]  = send_dataPID;
  packetData[9]  = 0xF1;                      //校验码

}




