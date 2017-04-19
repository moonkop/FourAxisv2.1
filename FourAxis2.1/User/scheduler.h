#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f4xx.h"

//循环计数结构体
typedef struct
{
    //循环运行完毕标志
    u8 check_flag;
    //代码在预定周期内没有运行完错误计数
    u8 err_flag;
    //不同周期的的执行任务独立计时
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
