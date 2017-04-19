#ifndef MPU_DMP_API_H
#define MPU_DMP_API_H

#include "stm32f4xx.h"

//提供给外部调用的变量
extern short gyro[3], accel[3];			//原始数据
extern float pitch,roll,yaw;						//姿态角

void gyro_data_ready_cb(void);
u8 mpu_dmp_init(void);
u8 dmp_getdata(void);

#endif
