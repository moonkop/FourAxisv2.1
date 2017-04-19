#include "control.h"
#include "mpu_dmp_api.h"
#include "deal_datapacket.h"
#include "Motor.h"
#include "LED.h"

#define GYRO_XISHU		(2000.0/65535.0)		//������GYROԭʼֵת��Ϊʵ�ʽ��ٶȵı���ϵ��

u8 Lock_State =0;//0�ɿؼ�����1�ɿؽ���
vs16 moto_pwm[4];						//�����ĸ������pwmռ�ձ�ֵ

vs16 gyro_X,gyro_Y,gyro_Z;						//x����ٶ�ֵ��y����ٶ�ֵ,z����ٶ�ֵ������ת�����ʵ��ֵ������ԭʼ���ݣ���һ�����˲����ã�

extern u8 dataPID;					//���ݰ�ʶ��PID
extern vu16 remoteControl[4];	//ң��������
extern u8 buttonFlag;//ң�ذ���

float Pitch_i,Roll_i,Yaw_i;              //������
float Pitch_old,Roll_old,Yaw_old;        //�Ƕȱ���
float Pitch_d,Roll_d,Yaw_d;              //΢����

volatile float RC_Pitch=0,RC_Roll=0,RC_Yaw=0;     //��������̬��

volatile float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;  //�⻷��������ڻ����ٶȸ���ֵ



//�⻷pid����
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


float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//���������ǽǶ� 

float e_I_Y,e_I_X,e_I_Z;						//�ڻ������ۼ�ƫ��ֵ
float e_X,e_Y,e_Z;						//����ƫ��

float pitch_core_kp_out,pitch_core_ki_out,pitch_core_kd_out;
float Roll_core_kp_out ,Roll_core_ki_out ,Roll_core_kd_out ;
float Yaw_core_kp_out  ,Yaw_core_ki_out  ,Yaw_core_kd_out  ;//�ڻ��������

float Pitch_core_out,Roll_core_out,Yaw_core_out;//�ڻ������



//�ڻ�PID����
float Pitch_core_kp=0.02;
float Pitch_core_ki=0;
float Pitch_core_kd=0;

float Roll_core_kp=0;
float Roll_core_ki=0;
float Roll_core_kd=0;

float Yaw_core_kp=0;
float Yaw_core_ki=0;
float Yaw_core_kd=0;
extern short gyro[3], accel[3];		  	//ԭʼ����
extern float pitch,roll,yaw;						//��̬��


//�⻷�Ƕȿ���
void CTRL_attitude(void)
{
	Pitch_i+=(RC_Pitch-pitch);
	//------pitch�����޷�-----------//
	if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
	//-------------Pitch΢��--------------------//
  Pitch_d=pitch-Pitch_old;
	//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(RC_Pitch-pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
	//------------Pitch����Ƕ�------------------//
  Pitch_old=pitch;
	/*********************************************************/       


  Roll_i+=(RC_Roll-roll);
	//-------------Roll�����޷�----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
	//-------------Roll΢��--------------------//
  Roll_d=roll-Roll_old;
	//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(RC_Roll-roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
	//------------Roll����Ƕ�------------------//
  Roll_old=roll;


	Yaw_i+=(RC_Yaw-yaw);
	//-------------Roll�����޷�----------------//
  if(Yaw_i>300) Yaw_i=300;
  else if(Yaw_i<-300) Yaw_i=-300;
	//-------------Yaw΢��--------------------//
  Yaw_d=gyro_Z-Yaw_old;
	//-------------Yaw  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(gyro_Z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
	//------------Yaw����Ƕ�------------------//
  Yaw_old=gyro_Z;
	
}


//�ڻ����ٶȿ���
void CTRL_angular_velocity(void)
{
	static u8 flag_Y=0,flag_X=0,flag_Z=0;					//�ڻ��������Ƿ��������ı�־
	
	/********************************
	��������ṹ
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
	
	
	/*******************λ��ʽPID����********************************/
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
	
	
	
	
	
	
	
	
	
	
	
//	/*******����y���x����ٶ�ƫ��*********/
//	e_Y=Pitch_shell_out-gyro_Y;
//	e_X=Roll_shell_out-gyro_X;
//	e_Z=Yaw_shell_out-gyro_Z;
	
	/**************���ַ���****************/
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
//	/**************�����޷�****************/
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
//	/*******************λ��ʽPID����********************************/
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





//��̬�Ǹ���
void Get_Angle(void)
{
	dmp_getdata();
	
	gyro_X = gyro[0]*GYRO_XISHU;
	gyro_Y = gyro[1]*GYRO_XISHU;
	gyro_Z = gyro[2]*GYRO_XISHU;
	

}



void Direction_Control(void)
{
	//����ң�������������ҷ���ֵ���ı�������pitch�����ǵ�ֵ
	RC_Pitch = -(remoteControl[2]-15);
	
	//����ң����������ǰ����ֵ���ı�������roll�����ǵ�ֵ
	RC_Roll = remoteControl[3]-15;
}


//pwm�������
void DealPwm(void)
{
	//���ж��Ƿ���key1��ָʾ���������ɻ������ڽ����ͼ�������ֹ���������
	if((!(buttonFlag & 0x01))&&!Lock_State){//�ɻ��Դ��ڼ���״̬(������˫����)

		//���⻷���ٶ����ֵ����Ϊ0����¼��ʱyaw�Ƕ�ֵ���Ҹ���RC_Yaw��ֵ
		Pitch_shell_out = 0;
		Roll_shell_out = 0;
		RC_Yaw = yaw;
		
		//��pwm���ֵ�ͻ����ۼ�ƫ��ֵȫ������Ϊ0 
		Pitch_core_out = 0;
		Roll_core_out = 0;
		Yaw_core_out = 0;
		e_I_X = 0;
		e_I_Y = 0;
		
		//������ֵ��Ϊ100
		remoteControl[1] = 100;
		return;
	}
	
	//���ɻ��Ѿ�����ʱ��������ֵС��110��˵���ɻ����ڴ���״̬����PID�������ֵ�ͻ����ۼ�����Ϊ0
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
	//ע��:�������Ϊs16,��Ϊmoto_pwm[i]Ϊ�з�����
	static s16 max = 200;
	static s16 min = 100;
	u8 i;
	
	//װ����������pwmֵ
	moto_pwm[0] = remoteControl[1]-Roll_core_out+Pitch_core_out+Yaw_core_out;//��׼���4
	moto_pwm[1] = remoteControl[1]+Roll_core_out+Pitch_core_out-Yaw_core_out;//��׼���3
	moto_pwm[2] = remoteControl[1]+Roll_core_out-Pitch_core_out+Yaw_core_out;//��׼���2
	moto_pwm[3] = remoteControl[1]-Roll_core_out-Pitch_core_out-Yaw_core_out;//��׼���1
	


	
	//PWM�޷�
	for(i=0;i<4;i++){
		if(moto_pwm[i] >= max){
			moto_pwm[i] = max;
		}else if(moto_pwm[i] <= min){
			moto_pwm[i] = min;
		}
	}
	

	
	Moto_PwmRflash(moto_pwm[3],moto_pwm[2],moto_pwm[1],moto_pwm[0]);
}


//�ж��Ƿ���ң����ʧȥ���ӣ�ʧȥ�������Զ�����
u8 dataPID_Last = 250;//������һ���յ������ݰ�PID
void Control_test(void)
{
	static u8 err_cnt=0;
	if(dataPID_Last == dataPID)     //���������Ѿ���ң����ʧȥ���ӣ���ȻҲ������������°�����
	{
			err_cnt++;
			if(err_cnt==3)//�������3�ζ����ж�ʧȥ���ӣ������ʧȥ������
			{
				remoteControl[1] = 100;            //���Ź���
				remoteControl[2] = 15;						//�������������ʱ��ǰ����ֵ����15
				remoteControl[3] = 15;						//�������������ʱ�����ҷ���ֵ����15
		
				LED_Connect_Fail();//��ң��������ʧ��   ָʾ��    	
				Lock_State=0;//����״̬
				err_cnt=0;
			}
	}
	else
	{
			err_cnt=0;
			dataPID_Last = dataPID;//������ң����ͨѶ���������ǵ�ԭ����ֵ
			LED_Locked();//������ӵ�û�н���  ָʾ��	
			Lock_State=0;//����״̬	
		
			if(buttonFlag&0x01)			 //����Ѿ�����
			{
				LED_Unlocked();//���ӳɹ��ҽ��� ָʾ��
		
				Lock_State=1;//����״̬
				
			}
		}
}

