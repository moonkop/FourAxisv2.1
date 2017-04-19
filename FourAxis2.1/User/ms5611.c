#include "ms5611.h"
#include "I2C.h"
#include "Systick.h"

#define MS5611_ADDR             0x77 // MS5611 address
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command

#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion

#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096

#define CMD_PROM_RD             0xA0 // Prom read command
#define MS5611_OSR				      0x08 // CMD_ADC_4096



//气压计计算单位 （mm 毫米）
int32_t baroAlt,baroAltOld;
//气压计计算速度 mm/s
float baro_alt_speed;
//
static uint16_t ms5611_prom8[8];
//读取的温度压强数据数组
static uint8_t t_rxbuf[3],p_rxbuf[3];




//温度
uint32_t ms5611_ut;
//气压
uint32_t ms5611_up;
//气压补偿
int32_t baro_Offset=0;
//气压计温度
float temperature_5611;


//气压计硬件故障
u8 hard_error_ms5611;



//气压计初始化
void MS5611_Init(void)
{
    //延时等待
    delay_ms(10);
    //气压计复位
    MS5611_Reset();
    //延时等待
    delay_ms(3);
    //读取气压计prom存储器，并判断故障
    hard_error_ms5611 =  MS5611_Read_Prom(ms5611_prom8);
    //开始读取温度
    MS5611_Start_T();
}




//气压计数据更新
int MS5611_Update(void)
{
    //气压计状态位
    static int state = 0;
    if (state)
    {
        //读取上次测量的气压	
        MS5611_Read_measure(p_rxbuf);
        //开始读取温度
        MS5611_Start_T();
        //气压计高度计算
        MS5611_BaroAltCalculate();
        //气压计状态位
        state = 0;
    }
    else
    {
        //读取上次测量的温度
        MS5611_Read_measure(t_rxbuf);
        //开始读取气压
        MS5611_Start_P();
        //气压计状态位
        state = 1;
    }
    //返回状态值
    return (state);
}





//气压计高度计算
int32_t pressure;
void MS5611_BaroAltCalculate(void)
{
    static u8 baro_start;
    int32_t temperature;
		int32_t	off2 = 0, sens2 = 0, delt;
//    int32_t pressure;
//   float alt_3;
    int32_t dT;
    int64_t off;
    int64_t sens;
    static vs32 sum_tmp_5611 = 0;

    //气压计温度原始数据 
    ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
    //气压计压强原始数据
    ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
    //气压计原始数据
    dT = ms5611_ut - ((uint32_t)ms5611_prom8[5] << 8);
    off = ((uint32_t)ms5611_prom8[2] << 16) + (((int64_t)dT * ms5611_prom8[4]) >> 7);
    sens = ((uint32_t)ms5611_prom8[1] << 15) + (((int64_t)dT * ms5611_prom8[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom8[6]) >> 23);

    //低于20度温度补偿
    if (temperature < 2000)
    {
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        //低于-15度补偿	
        if (temperature < -1500)
        {
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
        off  -= off2;
        sens -= sens2;
    }

    //计算出气压并转换为高度
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
 //   alt_3 = (101000 - pressure)/1000.0f;
 //   pressure = 0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
    //气压计计算高度 毫米单位
    baroAlt = pressure - baro_Offset;
    //气压计计算速度单位 mm/s
    baro_alt_speed += 5 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed );
    //保存数据下次使用
    baroAltOld = baroAlt;

    //第0-99次数据无效，用时？秒 11?
    if( baro_start < 100 )
    {
        //计算次数增加
        baro_start++;
        //气压计计算速度 清零
        baro_alt_speed = 0;
        //气压计计算高度 清零
        baroAlt = 0;
    }

    //气压计温度积分滤波
    temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
}

//气压计复位
void MS5611_Reset(void)
{
    I2C_WriteByteToSlave(MS5611_ADDR, CMD_RESET, 1);
}

//读取气压计prom存储器，并判断硬件故障
u8 MS5611_Read_Prom(uint16_t * ms5611_prom8)
{
    uint8_t rxbuf[2] = { 0, 0 };
    u8 check = 0;
    u8 i;

    for (i = 0; i < 8; i++)
    {
        check += I2C_ReadSomeDataFromSlave(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
        ms5611_prom8[i] = rxbuf[0] << 8 | rxbuf[1];
    }
    if(check==8)
        return 1;
    else
        return 0;
}

//读取测量数据
void MS5611_Read_measure(u8 * rxbuf)
{
    I2C_ReadSomeDataFromSlave( MS5611_ADDR, CMD_ADC_READ, 3, rxbuf );
}


//开始读取温度
void MS5611_Start_T(void)
{
    I2C_WriteByteToSlave(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1);
}


//开始读取大气压强
void MS5611_Start_P(void)
{
    I2C_WriteByteToSlave(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1);
}




