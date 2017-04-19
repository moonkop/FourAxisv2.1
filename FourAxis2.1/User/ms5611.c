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



//��ѹ�Ƽ��㵥λ ��mm ���ף�
int32_t baroAlt,baroAltOld;
//��ѹ�Ƽ����ٶ� mm/s
float baro_alt_speed;
//
static uint16_t ms5611_prom8[8];
//��ȡ���¶�ѹǿ��������
static uint8_t t_rxbuf[3],p_rxbuf[3];




//�¶�
uint32_t ms5611_ut;
//��ѹ
uint32_t ms5611_up;
//��ѹ����
int32_t baro_Offset=0;
//��ѹ���¶�
float temperature_5611;


//��ѹ��Ӳ������
u8 hard_error_ms5611;



//��ѹ�Ƴ�ʼ��
void MS5611_Init(void)
{
    //��ʱ�ȴ�
    delay_ms(10);
    //��ѹ�Ƹ�λ
    MS5611_Reset();
    //��ʱ�ȴ�
    delay_ms(3);
    //��ȡ��ѹ��prom�洢�������жϹ���
    hard_error_ms5611 =  MS5611_Read_Prom(ms5611_prom8);
    //��ʼ��ȡ�¶�
    MS5611_Start_T();
}




//��ѹ�����ݸ���
int MS5611_Update(void)
{
    //��ѹ��״̬λ
    static int state = 0;
    if (state)
    {
        //��ȡ�ϴβ�������ѹ	
        MS5611_Read_measure(p_rxbuf);
        //��ʼ��ȡ�¶�
        MS5611_Start_T();
        //��ѹ�Ƹ߶ȼ���
        MS5611_BaroAltCalculate();
        //��ѹ��״̬λ
        state = 0;
    }
    else
    {
        //��ȡ�ϴβ������¶�
        MS5611_Read_measure(t_rxbuf);
        //��ʼ��ȡ��ѹ
        MS5611_Start_P();
        //��ѹ��״̬λ
        state = 1;
    }
    //����״ֵ̬
    return (state);
}





//��ѹ�Ƹ߶ȼ���
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

    //��ѹ���¶�ԭʼ���� 
    ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
    //��ѹ��ѹǿԭʼ����
    ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
    //��ѹ��ԭʼ����
    dT = ms5611_ut - ((uint32_t)ms5611_prom8[5] << 8);
    off = ((uint32_t)ms5611_prom8[2] << 16) + (((int64_t)dT * ms5611_prom8[4]) >> 7);
    sens = ((uint32_t)ms5611_prom8[1] << 15) + (((int64_t)dT * ms5611_prom8[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom8[6]) >> 23);

    //����20���¶Ȳ���
    if (temperature < 2000)
    {
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        //����-15�Ȳ���	
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

    //�������ѹ��ת��Ϊ�߶�
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
 //   alt_3 = (101000 - pressure)/1000.0f;
 //   pressure = 0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
    //��ѹ�Ƽ���߶� ���׵�λ
    baroAlt = pressure - baro_Offset;
    //��ѹ�Ƽ����ٶȵ�λ mm/s
    baro_alt_speed += 5 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed );
    //���������´�ʹ��
    baroAltOld = baroAlt;

    //��0-99��������Ч����ʱ���� 11?
    if( baro_start < 100 )
    {
        //�����������
        baro_start++;
        //��ѹ�Ƽ����ٶ� ����
        baro_alt_speed = 0;
        //��ѹ�Ƽ���߶� ����
        baroAlt = 0;
    }

    //��ѹ���¶Ȼ����˲�
    temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
}

//��ѹ�Ƹ�λ
void MS5611_Reset(void)
{
    I2C_WriteByteToSlave(MS5611_ADDR, CMD_RESET, 1);
}

//��ȡ��ѹ��prom�洢�������ж�Ӳ������
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

//��ȡ��������
void MS5611_Read_measure(u8 * rxbuf)
{
    I2C_ReadSomeDataFromSlave( MS5611_ADDR, CMD_ADC_READ, 3, rxbuf );
}


//��ʼ��ȡ�¶�
void MS5611_Start_T(void)
{
    I2C_WriteByteToSlave(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1);
}


//��ʼ��ȡ����ѹǿ
void MS5611_Start_P(void)
{
    I2C_WriteByteToSlave(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1);
}




