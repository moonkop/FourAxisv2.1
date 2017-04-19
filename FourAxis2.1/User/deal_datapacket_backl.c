#include "deal_datapacket.h"
#include "led.h"
#include "DEBUG.h"
u8 dataPID;												//������յ������ݰ�ʶ��PIDֵ
u8 buttonFlag = 0x00;									//ָʾ�ĸ��������£�0x00��ʾ4��������û�б�����
vu16 remoteControl[4];									//����ң�����н��յ�����������ƴװ

u8 packetData[10];            //����������ң����������
u8 send_dataPID = 0;	        //���͵����ݰ�ʶ��PIDֵ

extern u8 rxbuf[12];									//���ݽ��ջ�����
extern float pitch,roll,yaw;          //��̬��
//===========================================================================================


/******************************************************************************************
������յ���ң�������ݣ������ݷ��͹��������ݽ�����Ӧ�Ĵ���

���������ͨѶЭ����н��룬���ֽ�Ϊ��λ
ǰ����-����MASK--ADC1��8--ADC1��8--ADC2��8--ADC2��8--ADC3��8--ADC3��8--ADC4��8--ADC4��8--���ݰ���ʶ--У����0xa5
����:ǰ����ֻ��0x01��0x08�ű�ʾ��Ч�����ݰ�,0x01��ʾ�����ݰ�����ADC������ɴ���,0x08��ʾ����ң��������������
���ݱ���ʶ�����Ƿ���ͬһ���ݰ�������(��Ҫ���ڵ�ң���ź��ж�ʱ���ɻ���ʼ�Զ�����)
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
	if(rxbuf[0] & 0x01){								//�����ݰ�����ң������ADC�������ʱ��������ʱ
		remoteControl[0] = rxbuf[3]<<8|rxbuf[2];		//ADC2
		remoteControl[1] = rxbuf[5]<<8|rxbuf[4];		//ADC1
		remoteControl[2] = rxbuf[7]<<8|rxbuf[6];		//ADC4
		remoteControl[3] = rxbuf[9]<<8|rxbuf[8];		//ADC3
	}else if(rxbuf[0] & 0x08){							//�����ݰ�����ң�����İ�����������ʱ
		buttonFlag = rxbuf[1];
	}
	
	//�����ݰ�ʶ��PIDֵȡ��������֮ǰ��ֵ���Ա�ʾ�ź���������
	dataPID = rxbuf[10];
}


/****************************************************************************
�����������ң����������

���������ͨѶЭ����д����(��̬��Ҫת��u32λ)
ǰ����--������--pitch��8λ--pitch��8λ--roll��8λ--roll��8λ--yaw��8λ--yaw��8λ--���ݰ���ʶ--У����
���У�ǰ����ֻ��0xAA�ű�ʾ��Ч�����ݰ�

****************************************************************************/
void PackData(u8 Lock_State)
{
	
	if(send_dataPID>=200){
		send_dataPID = 0;
	}else{
		send_dataPID++;
	}
	
	//===ֱ�Ӳ���ָ������ڴ��е���ֵ��16λת��8λ���ٶȿ죬�Ҳ��ᷢ�����Ƚ�ȡ������ע�⣺stm32��С�˵�ַ
	packetData[0]  = 0xAA;
	packetData[1]  = Lock_State;
  packetData[2]  = *((u8*)&pitch);
  packetData[3]  = *(((u8*)&pitch)+1);
  packetData[4]  = *((u8*)&roll);
  packetData[5]  = *(((u8*)&roll)+1);
  packetData[6]  = *((u8*)&yaw);
  packetData[7]  = *(((u8*)&yaw)+1);
  packetData[8]  = send_dataPID;
  packetData[9]  = 0xF1;                      //У����

}




