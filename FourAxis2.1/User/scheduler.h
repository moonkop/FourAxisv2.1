#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f4xx.h"

//ѭ�������ṹ��
typedef struct
{
    //ѭ��������ϱ�־
    u8 check_flag;
    //������Ԥ��������û��������������
    u8 err_flag;
    //��ͬ���ڵĵ�ִ�����������ʱ
    s16 cnt_2ms;
    s16 cnt_5ms;
    s16 cnt_10ms;
    s16 cnt_20ms;
    s16 cnt_50ms;
    s16 cnt_500ms;
} loop_t;

extern void Main_Loop(void);

extern void Call_Loop_timer(void);


#endif 
