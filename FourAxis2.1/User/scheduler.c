#include "scheduler.h"
#include "Usart.h"
#include "deal_datapacket.h"
#include "ANO_DT.h"
#include "control.h"
#include "mpu_dmp_api.h"
#include "ms5611.h"
//ѭ�������ṹ��
loop_t loop;

void Duty_1ms()
{
	Data_transfer();
}

void Duty_2ms()
{
	//�ڻ����ٶȿ���
	CTRL_angular_velocity();
	
	Direction_Control();//�������
	
	DealPwm();//����pwm
  Set_Pwm();//����pwm�����
}

void Duty_5ms()
{
		//������̬��
		gyro_data_ready_cb();
		Get_Angle();
		
		//�⻷�Ƕȿ���
	//	CTRL_attitude();
	
}


extern u8 Lock_State;
extern u8 packetData[10];            //����������ң����������
void Duty_10ms()
{
	
}

void Duty_20ms()
{
	
}


void Duty_50ms()
{
//	static int ax=0;
//	
//	if(MS5611_Update())
//	{
//		ax++;
//		if(ax==1)
//		{
//				printf("%d\n",pressure);
//				ax=0;
//		}
//	}
	
	//printf("Duty_50ms!!!");
	//Usart1_Send(rxbuf,12);
}


void Duty_500ms()
{
	//�ж��Ƿ���ң����ʧȥ����
	Control_test();
	
	//�����ң���������� ������״̬�Լ���̬��
	PackData(Lock_State);
//	Usart1_Send(packetData,10);
	Usart2_Send(packetData,10);
	
}


void Main_Loop()
{
		//ѭ������Ϊ1ms
    if( loop.check_flag == 1 )
    {
				Duty_1ms();//����Ϊ1ms������
			
				//�ж�ÿ����ͬ���ڵ�ִ������ִ������
				if( loop.cnt_2ms >= 2 )
        {
            loop.cnt_2ms = 0;
            Duty_2ms();   	//����Ϊ1ms������
        }
        if( loop.cnt_5ms >= 5 )
        {
            loop.cnt_5ms = 0;
            Duty_5ms();    //����Ϊ5ms������
        }
        if( loop.cnt_10ms >= 10 )
        {
            loop.cnt_10ms = 0;
            Duty_10ms();   //����Ϊ10ms������
        }
        if( loop.cnt_20ms >= 20 )
        {
            loop.cnt_20ms = 0;
            Duty_20ms();   //����Ϊ20ms������
        }
        if( loop.cnt_50ms >= 50 )
				{
                    loop.cnt_50ms = 0;
            Duty_50ms();   //����Ϊ50ms������
        }
				if(loop.cnt_500ms>=250)
				{
						loop.cnt_500ms=0;
						Duty_500ms();//����Ϊ500ms������
				}
	
        //ѭ��������ϣ���־����
        loop.check_flag = 0;
    }
}


void Call_Loop_timer()
{
    //��ͬ���ڵ�ִ�����������ʱ
    loop.cnt_2ms++;
    loop.cnt_5ms++;
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;
    loop.cnt_500ms++;

    //���������Ԥ��������û��ִ����
    if( loop.check_flag == 1)
			loop.err_flag ++;//�����������	
    //ѭ����ʼ��־��1
    else loop.check_flag = 1;

}



