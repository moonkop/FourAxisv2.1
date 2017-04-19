#include "stm32f4xx_it.h"
#include "mpu_dmp_api.h"
#include "Usart.h"
#include "ANO_DT.h" 
#include "scheduler.h"
#include "deal_datapacket.h"
#include "control.h"




//收到上位机的PID调整
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		u8 res=USART_ReceiveData(USART1);
		ANO_DT_Data_Receive_Prepare(res);
		
		
	}
}





extern u8 recstatu;        //表示是否处于一个正在接收的状态
extern u8 ccnt;             //计数
extern u8 packerflag;			 //是否接收到一个完整的数据包标志
extern u8 rxbuf[RX_WIDTH]; //缓冲区

//接收遥控数据
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)
	{
		
		u8 res;
		res=USART_ReceiveData(USART2);
		importData(res);
//		if((res==0x01||res==0x08)&&ccnt==0)//判断是否为前导码
//		{
//			recstatu=1;
//			
//			packerflag=0;
//			rxbuf[ccnt++] = res;
//			return;
//		}
//		if(ccnt==11)   //检测是否为校验码
//		{
//			recstatu=0;
//			packerflag=1;
//			rxbuf[ccnt++] = res;	
//		
////					for(int i =0 ;i<ccnt;i++)
////					{
////						printf("%x ",rxbuf[i]);
////					}
////					printf("\n");
//			
//			UnpackData();//收到完整的包后解包
//			
//			ccnt=0;
//			return;
//		}
//		if(ccnt>11)
//		{
//			recstatu=0;
//				ccnt=0;
//			
//		}
//		if(recstatu==1)  //是否处于接收数据包的状态
//		{
//			 rxbuf[ccnt++] = res;
//		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}



void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4,USART_IT_RXNE)!= RESET)
	{
		
		u8 res;
		res=USART_ReceiveData(UART4);
		printf("%x ",res);
		
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);
	}
}

	
extern u8 Init_Finish;
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		//初始化未完成，不进行大循环
		if( ! Init_Finish) return;
    Call_Loop_timer();
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

//DMP处理完数据产生的中断
//已取消中断
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7)!=RESET)
	{
//		gyro_data_ready_cb();
//		Get_Angle();//更新姿态角
//		Direction_Control();//方向控制
//		Control_PID();//PID
//		
//		DealPwm();//处理pwm
//    Set_Pwm();//分配pwm给电机
		
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	
}



