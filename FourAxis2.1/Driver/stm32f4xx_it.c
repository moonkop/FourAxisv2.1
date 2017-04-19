#include "stm32f4xx_it.h"
#include "mpu_dmp_api.h"
#include "Usart.h"
#include "ANO_DT.h" 
#include "scheduler.h"
#include "deal_datapacket.h"
#include "control.h"




//�յ���λ����PID����
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		u8 res=USART_ReceiveData(USART1);
		ANO_DT_Data_Receive_Prepare(res);
		
		
	}
}





extern u8 recstatu;        //��ʾ�Ƿ���һ�����ڽ��յ�״̬
extern u8 ccnt;             //����
extern u8 packerflag;			 //�Ƿ���յ�һ�����������ݰ���־
extern u8 rxbuf[RX_WIDTH]; //������

//����ң������
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)
	{
		
		u8 res;
		res=USART_ReceiveData(USART2);
		importData(res);
//		if((res==0x01||res==0x08)&&ccnt==0)//�ж��Ƿ�Ϊǰ����
//		{
//			recstatu=1;
//			
//			packerflag=0;
//			rxbuf[ccnt++] = res;
//			return;
//		}
//		if(ccnt==11)   //����Ƿ�ΪУ����
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
//			UnpackData();//�յ������İ�����
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
//		if(recstatu==1)  //�Ƿ��ڽ������ݰ���״̬
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
		//��ʼ��δ��ɣ������д�ѭ��
		if( ! Init_Finish) return;
    Call_Loop_timer();
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

//DMP���������ݲ������ж�
//��ȡ���ж�
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7)!=RESET)
	{
//		gyro_data_ready_cb();
//		Get_Angle();//������̬��
//		Direction_Control();//�������
//		Control_PID();//PID
//		
//		DealPwm();//����pwm
//    Set_Pwm();//����pwm�����
		
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	
}



