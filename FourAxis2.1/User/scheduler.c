#include "scheduler.h"
#include "Usart.h"
#include "deal_datapacket.h"
#include "ANO_DT.h"
#include "control.h"
#include "mpu_dmp_api.h"
#include "ms5611.h"
//循环计数结构体
loop_t loop;

void Duty_1ms()
{
	Data_transfer();
}

void Duty_2ms()
{
	//内环角速度控制
	CTRL_angular_velocity();
	
	Direction_Control();//方向控制
	
	DealPwm();//处理pwm
  Set_Pwm();//分配pwm给电机
}

void Duty_5ms()
{
		//更新姿态角
		gyro_data_ready_cb();
		Get_Angle();
		
		//外环角度控制
	//	CTRL_attitude();
	
}


extern u8 Lock_State;
extern u8 packetData[10];            //打包后待发给遥控器的数据
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
	//判断是否与遥控器失去连接
	Control_test();
	
	//打包给遥控器的数据 ：解锁状态以及姿态角
	PackData(Lock_State);
//	Usart1_Send(packetData,10);
	Usart2_Send(packetData,10);
	
}


void Main_Loop()
{
		//循环周期为1ms
    if( loop.check_flag == 1 )
    {
				Duty_1ms();//周期为1ms的任务
			
				//判断每个不同周期的执行任务执行条件
				if( loop.cnt_2ms >= 2 )
        {
            loop.cnt_2ms = 0;
            Duty_2ms();   	//周期为1ms的任务
        }
        if( loop.cnt_5ms >= 5 )
        {
            loop.cnt_5ms = 0;
            Duty_5ms();    //周期为5ms的任务
        }
        if( loop.cnt_10ms >= 10 )
        {
            loop.cnt_10ms = 0;
            Duty_10ms();   //周期为10ms的任务
        }
        if( loop.cnt_20ms >= 20 )
        {
            loop.cnt_20ms = 0;
            Duty_20ms();   //周期为20ms的任务
        }
        if( loop.cnt_50ms >= 50 )
				{
                    loop.cnt_50ms = 0;
            Duty_50ms();   //周期为50ms的任务
        }
				if(loop.cnt_500ms>=250)
				{
						loop.cnt_500ms=0;
						Duty_500ms();//周期为500ms的任务
				}
	
        //循环运行完毕，标志清零
        loop.check_flag = 0;
    }
}


void Call_Loop_timer()
{
    //不同周期的执行任务独立计时
    loop.cnt_2ms++;
    loop.cnt_5ms++;
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;
    loop.cnt_500ms++;

    //如果代码在预定周期内没有执行完
    if( loop.check_flag == 1)
			loop.err_flag ++;//错误次数计数	
    //循环开始标志置1
    else loop.check_flag = 1;

}



