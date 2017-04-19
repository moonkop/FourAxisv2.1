#include "control.h"
#include "mpu_dmp_api.h"
#include "deal_datapacket.h"
#include "Motor.h"
#include "LED.h"

#define GYRO_XISHU		(2000.0/65535.0)		//用来将GYRO原始值转换为实际角速度的比例系数

u8 Lock_State =0;//0飞控加锁，1飞控解锁
vs16 moto_pwm[4];						//保存四个电机的pwm占空比值

vs16 gyro_X,gyro_Y,gyro_Z;						//x轴角速度值，y轴角速度值,z轴角速度值（这是转换后的实际值，不用原始数据，起到一定的滤波作用）

extern u8 dataPID;					//数据包识别PID
extern vu16 remoteControl[4];	//遥控器数据
extern u8 buttonFlag;//遥控按键

float Pitch_i,Roll_i,Yaw_i;              //积分项
float Pitch_old,Roll_old,Yaw_old;        //角度保存
float Pitch_d,Roll_d,Yaw_d;              //微分项

volatile float RC_Pitch=0,RC_Roll=0,RC_Yaw=0;     //期望的姿态角

volatile float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;  //外环总输出，内环角速度给定值



//外环pid参数
float Pitch_shell_kp=0;
float Pitch_shell_ki=0;
float Pitch_shell_kd=0;
/*********************************/
float Roll_shell_kp=0;              
float Roll_shell_ki=0;
float Roll_shell_kd=0; 
/*********************************/        
float Yaw_shell_kp=0;              
float Yaw_shell_ki=0;
float Yaw_shell_kd=0;


float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//保存陀螺仪角度 

float e_I_Y,e_I_X,e_I_Z;						//内环积分累计偏差值
float e_X,e_Y,e_Z;						//本次偏差

float pitch_core_kp_out,pitch_core_ki_out,pitch_core_kd_out;
float Roll_core_kp_out ,Roll_core_ki_out ,Roll_core_kd_out ;
float Yaw_core_kp_out  ,Yaw_core_ki_out  ,Yaw_core_kd_out  ;//内环单项输出

float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出



//内环PID参数
float Pitch_core_kp=0.02;
float Pitch_core_ki=0;
float Pitch_core_kd=0;

float Roll_core_kp=0;
float Roll_core_ki=0;
float Roll_core_kd=0;

float Yaw_core_kp=0;
float Yaw_core_ki=0;
float Yaw_core_kd=0;
extern short gyro[3], accel[3];		  	//原始数据
extern float pitch,roll,yaw;						//姿态角


//外环角度控制
void CTRL_attitude(void)
{
	Pitch_i+=(RC_Pitch-pitch);
	//------pitch积分限幅-----------//
	if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
	//-------------Pitch微分--------------------//
  Pitch_d=pitch-Pitch_old;
	//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(RC_Pitch-pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
	//------------Pitch保存角度------------------//
  Pitch_old=pitch;
	/*********************************************************/       


  Roll_i+=(RC_Roll-roll);
	//-------------Roll积分限幅----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
	//-------------Roll微分--------------------//
  Roll_d=roll-Roll_old;
	//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(RC_Roll-roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
	//------------Roll保存角度------------------//
  Roll_old=roll;


	Yaw_i+=(RC_Yaw-yaw);
	//-------------Roll积分限幅----------------//
  if(Yaw_i>300) Yaw_i=300;
  else if(Yaw_i<-300) Yaw_i=-300;
	//-------------Yaw微分--------------------//
  Yaw_d=gyro_Z-Yaw_old;
	//-------------Yaw  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(gyro_Z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
	//------------Yaw保存角度------------------//
  Yaw_old=gyro_Z;
	
}


//内环角速度控制
void CTRL_angular_velocity(void)
{
	static u8 flag_Y=0,flag_X=0,flag_Z=0;					//内环积分项是否参与运算的标志
	
	/********************************
	调整程序结构
	*********************************/
	e_Y=RC_Pitch-gyro_Y;
	//e_X=Roll_shell_out-gyro_X;
//	e_Z=Yaw_shell_out-gyro_Z;

		if(e_Y>=150.0f||e_Y<=-150.0f){
		flag_Y = 0;
	}else{
		flag_Y = 1;
		e_I_Y += e_Y;
	}
//	
//	if(e_X>=150.0f||e_X<=150.0f){
//		flag_X = 0;
//	}else{
//		flag_X = 1;
//		e_I_X += e_X;
//	}
//	
//	if(e_Z>=150.0f||e_Z<=150.0f){
//		flag_Z = 0;
//	}else{
//		flag_Z = 1;
//		e_I_Z += e_Z;
//	}
	

	if(e_I_Y>1000)
		e_I_Y=1000;
	if(e_I_Y<-1000)
		e_I_Y=-1000;
//	
//	if(e_I_X>1000)
//		e_I_X=1000;             					
//	if(e_I_X<-1000)	
//		e_I_X=-1000;        
//	
//	if(e_I_Z>1000)
//		e_I_Z=1000;             					
//	if(e_I_Z<-1000)	
//		e_I_Z=-1000; 
//	
	
	
	/*******************位置式PID运算********************************/
  pitch_core_kp_out = Pitch_core_kp * (RC_Pitch - gyro_Y );
	pitch_core_ki_out = Pitch_core_ki * flag_Y * e_I_Y;
  pitch_core_kd_out = Pitch_core_kd * (gyro_Y   - Gyro_radian_old_y);

  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  - gyro_X );
	Roll_core_ki_out  = Roll_core_ki * flag_X * e_I_X;
  Roll_core_kd_out  = Roll_core_kd  * (gyro_X   - Gyro_radian_old_x);

  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  - gyro_Z);
	Yaw_core_ki_out  = Yaw_core_ki * flag_Z * e_I_Z;
  Yaw_core_kd_out  = Yaw_core_kd  * (gyro_Z   - Gyro_radian_old_z);


  Pitch_core_out = pitch_core_kp_out + pitch_core_ki_out + pitch_core_kd_out;
  Roll_core_out  = 0;
  Yaw_core_out   = 0;
	
	
	
	
	
	
	
	
	
	
	
//	/*******计算y轴和x轴角速度偏差*********/
//	e_Y=Pitch_shell_out-gyro_Y;
//	e_X=Roll_shell_out-gyro_X;
//	e_Z=Yaw_shell_out-gyro_Z;
	
	/**************积分分离****************/
//	if(e_Y>=150.0f||e_Y<=-150.0f){
//		flag_Y = 0;
//	}else{
//		flag_Y = 1;
//		e_I_Y += e_Y;
//	}
//	
//	if(e_X>=150.0f||e_X<=150.0f){
//		flag_X = 0;
//	}else{
//		flag_X = 1;
//		e_I_X += e_X;
//	}
//	
//	if(e_Z>=150.0f||e_Z<=150.0f){
//		flag_Z = 0;
//	}else{
//		flag_Z = 1;
//		e_I_Z += e_Z;
//	}
//	
//	
//	
//	/**************积分限幅****************/
//	if(e_I_Y>1000)
//		e_I_Y=1000;
//	if(e_I_Y<-1000)
//		e_I_Y=-1000;
//	
//	if(e_I_X>1000)
//		e_I_X=1000;             					
//	if(e_I_X<-1000)	
//		e_I_X=-1000;        
//	
//	if(e_I_Z>1000)
//		e_I_Z=1000;             					
//	if(e_I_Z<-1000)	
//		e_I_Z=-1000;    
//	
//	
//	/*******************位置式PID运算********************************/
//  pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out - gyro_Y );
//	pitch_core_ki_out = Pitch_core_ki * flag_Y * e_I_Y;
//  pitch_core_kd_out = Pitch_core_kd * (gyro_Y   - Gyro_radian_old_y);

//  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  - gyro_X );
//	Roll_core_ki_out  = Roll_core_ki * flag_X * e_I_X;
//  Roll_core_kd_out  = Roll_core_kd  * (gyro_X   - Gyro_radian_old_x);

//  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  - gyro_Z);
//	Yaw_core_ki_out  = Yaw_core_ki * flag_Z * e_I_Z;
//  Yaw_core_kd_out  = Yaw_core_kd  * (gyro_Z   - Gyro_radian_old_z);


//  Pitch_core_out = pitch_core_kp_out + pitch_core_ki_out + pitch_core_kd_out;
//  Roll_core_out  = Roll_core_kp_out  + Roll_core_ki_out + Roll_core_kd_out;
//  Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_ki_out + Yaw_core_kd_out;

  Gyro_radian_old_y = gyro_X;
  Gyro_radian_old_x = gyro_Y;
  Gyro_radian_old_z = gyro_Z;   
}





//姿态角更新
void Get_Angle(void)
{
	dmp_getdata();
	
	gyro_X = gyro[0]*GYRO_XISHU;
	gyro_Y = gyro[1]*GYRO_XISHU;
	gyro_Z = gyro[2]*GYRO_XISHU;
	

}



void Direction_Control(void)
{
	//根据遥控器传来的左右方向值，改变期望的pitch俯仰角的值
	RC_Pitch = -(remoteControl[2]-15);
	
	//根据遥控器传来的前后方向值，改变期望的roll翻滚角的值
	RC_Roll = remoteControl[3]-15;
}


//pwm输出限速
void DealPwm(void)
{
	//先判断是否按下key1，指示即将启动飞机，用于解锁和加锁，防止油门误操作
	if((!(buttonFlag & 0x01))&&!Lock_State){//飞机仍处于加锁状态(怕死，双保险)

		//将外环角速度输出值重置为0，记录此时yaw角度值并且覆盖RC_Yaw的值
		Pitch_shell_out = 0;
		Roll_shell_out = 0;
		RC_Yaw = yaw;
		
		//将pwm输出值和积分累计偏差值全部重置为0 
		Pitch_core_out = 0;
		Roll_core_out = 0;
		Yaw_core_out = 0;
		e_I_X = 0;
		e_I_Y = 0;
		
		//将油门值改为100
		remoteControl[1] = 100;
		return;
	}
	
	//当飞机已经解锁时，但油门值小于110，说明飞机处于待飞状态，把PID运算输出值和积分累计重置为0
	if(remoteControl[1]<=110){
		Pitch_shell_out = 0;
		Roll_shell_out = 0;
		RC_Yaw = yaw;

		Pitch_core_out = 0;
		Roll_core_out = 0;
		Yaw_core_out = 0;
		e_I_X = 0;
		e_I_Y = 0;
		
	}
}

void Set_Pwm(void)
{
	//注意:这里必须为s16,因为moto_pwm[i]为有符号数
	static s16 max = 200;
	static s16 min = 100;
	u8 i;
	
	//装配给各电机的pwm值
	moto_pwm[0] = remoteControl[1]-Roll_core_out+Pitch_core_out+Yaw_core_out;//标准电机4
	moto_pwm[1] = remoteControl[1]+Roll_core_out+Pitch_core_out-Yaw_core_out;//标准电机3
	moto_pwm[2] = remoteControl[1]+Roll_core_out-Pitch_core_out+Yaw_core_out;//标准电机2
	moto_pwm[3] = remoteControl[1]-Roll_core_out-Pitch_core_out-Yaw_core_out;//标准电机1
	


	
	//PWM限幅
	for(i=0;i<4;i++){
		if(moto_pwm[i] >= max){
			moto_pwm[i] = max;
		}else if(moto_pwm[i] <= min){
			moto_pwm[i] = min;
		}
	}
	

	
	Moto_PwmRflash(moto_pwm[3],moto_pwm[2],moto_pwm[1],moto_pwm[0]);
}


//判断是否与遥控器失去连接，失去连接则自动降落
u8 dataPID_Last = 250;//保存上一次收到的数据包PID
void Control_test(void)
{
	static u8 err_cnt=0;
	if(dataPID_Last == dataPID)     //表明四轴已经和遥控器失去连接，当然也可能是误包导致包舍弃
	{
			err_cnt++;
			if(err_cnt==3)//如果连续3次都是判定失去连接，则真的失去连接了
			{
				remoteControl[1] = 100;            //油门归零
				remoteControl[2] = 15;						//当四轴脱离控制时，前后方向值给定15
				remoteControl[3] = 15;						//当四轴脱离控制时，左右方向值给定15
		
				LED_Connect_Fail();//与遥控器连接失败   指示灯    	
				Lock_State=0;//加锁状态
				err_cnt=0;
			}
	}
	else
	{
			err_cnt=0;
			dataPID_Last = dataPID;//四轴与遥控器通讯正常，覆盖掉原来的值
			LED_Locked();//如果连接但没有解锁  指示灯	
			Lock_State=0;//加锁状态	
		
			if(buttonFlag&0x01)			 //如果已经解锁
			{
				LED_Unlocked();//连接成功且解锁 指示灯
		
				Lock_State=1;//解锁状态
				
			}
		}
}

