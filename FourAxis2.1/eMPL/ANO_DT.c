
//#include "data_transfer.h"
#include "Usart.h"
//#include "imu.h"
//#include "mpu6050.h"
//#include "ak8975.h"
//#include "ms5611.h"
//#include "rc.h"
//#include "ctrl.h"
//#include "time.h"
//#include "usbd_user_hid.h"
#include "ANO_DT.h" 
#include "mpu_dmp_api.h"
#include "Systick.h"
#include "control.h"

/////////////////////////////////////////////////////////////////////////////////////
//???????,?????1????????,??int16?float?,????????????????
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	

dt_flag_t f;					//等待发送数据的标志
u8 data_to_send[50];	//发送数据缓存数组

extern float pitch,roll,yaw;		
extern vs16 moto_pwm[4];	

extern u8 Lock_State;//飞控加锁状态
//外环pid参数
extern float Pitch_shell_kp;
extern float Pitch_shell_ki;
extern float Pitch_shell_kd;
/*********************************/
extern float Roll_shell_kp;              
extern float Roll_shell_ki;
extern float Roll_shell_kd; 
/*********************************/        
extern float Yaw_shell_kp;              
extern float Yaw_shell_ki;
extern float Yaw_shell_kd;


//内环PID参数
extern float Pitch_core_kp;
extern float Pitch_core_ki;
extern float Pitch_core_kd;

extern float Roll_core_kp;
extern float Roll_core_ki;
extern float Roll_core_kd;

extern float Yaw_core_kp;
extern float Yaw_core_ki;
extern float Yaw_core_kd;

void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;


	ANO_DT_Send_Data(data_to_send, 7);

}

extern  s32 err_cnt;
void Data_transfer(void)
{
	//	s32 alt=25000;
		u8 fly_model=1;

	
		static int cnt = 0;
    //cnt是1-10000的数据
    if(++cnt>10000) cnt = 1;
    //发送姿态数据，周期49ms
    if((cnt % 49) == 0)
        f.send_status = 1;
		
    //发送电机pwm数据，周期ms
		if((cnt % 59) == 0)
        f.send_motopwm = 1;
		
//		if((cnt%90)==0)
//		{
//			f.send_pid1=1;
//			f.send_pid2=1;
//		}
//		
		if(f.send_status)
    {
        f.send_status = 0;
				ANO_DT_Send_Status(pitch,roll,yaw,err_cnt*100,fly_model,Lock_State);
    }
		if(f.send_motopwm)
    {
        f.send_motopwm = 0;
				ANO_DT_Send_MotoPWM((moto_pwm[0]-100)*10,(moto_pwm[1]-100)*10,(moto_pwm[2]-100)*10,(moto_pwm[3]-100)*10,0,0,0,0);
    }
		if(f.send_pid1)
		{
			f.send_pid1=0;
			ANO_DT_Send_PID(1,Pitch_shell_kp,Pitch_shell_ki,Pitch_shell_kd,Roll_shell_kp,Roll_shell_ki,Roll_shell_kd,Yaw_shell_kp,Yaw_shell_ki,Yaw_shell_kd);
		}
		if(f.send_pid2)
		{
			f.send_pid2=0;
			ANO_DT_Send_PID(2,Pitch_core_kp,Pitch_core_ki,Pitch_core_kd,Roll_core_kp,Roll_core_ki,Roll_core_kd,Yaw_core_kp,Yaw_core_ki,Yaw_core_kd);
		}
}


void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}





void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

	
}

void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}


void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????
	
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
//			f.send_pid3 = 1;
//			f.send_pid4 = 1;
//			f.send_pid5 = 1;
//			f.send_pid6 = 1;
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
	{
			Pitch_shell_kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
			Pitch_shell_ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
			Pitch_shell_kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			Roll_shell_kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
			Roll_shell_ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
			Roll_shell_kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
			Yaw_shell_kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
			Yaw_shell_ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
			Yaw_shell_kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );

			ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
			Pitch_core_kp 	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
			Pitch_core_ki 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
			Pitch_core_kd 	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			Roll_core_kp 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
			Roll_core_ki 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
			Roll_core_kd	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
			Yaw_core_kp	  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
			Yaw_core_ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
			Yaw_core_kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
			ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{	
		
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
		
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
	
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
	
	}
}


void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_Send_Data(u8 *data_to_send, u8 len)
{
	Usart1_Send(data_to_send, len);
}



