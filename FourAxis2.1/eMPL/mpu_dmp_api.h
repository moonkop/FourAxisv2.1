#ifndef MPU_DMP_API_H
#define MPU_DMP_API_H

#include "stm32f4xx.h"

//�ṩ���ⲿ���õı���
extern short gyro[3], accel[3];			//ԭʼ����
extern float pitch,roll,yaw;						//��̬��

void gyro_data_ready_cb(void);
u8 mpu_dmp_init(void);
u8 dmp_getdata(void);

#endif
