/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      motion_driver_test.c
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx.h"
#include "I2C.h"
#include "mpu_dmp_api.h"


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)								//����λ���ϱ����ټ����ݵı�־
#define PRINT_GYRO      (0x02)              	//����λ���ϱ����������ݵı�־
#define PRINT_QUAT      (0x04)              	//����λ���ϱ���Ԫ�����ݵı�־
                                            
#define ACCEL_ON        (0x01)              	//ָʾ���ټ��Ѿ��򿪵ı�־
#define GYRO_ON         (0x02)              	//ָʾ�������Ѿ��򿪵ı�־
                                            
#define MOTION          (0)                 	//ָʾMPU�����˶�״̬
#define NO_MOTION       (1)                 	//ָʾMPU���ھ�ֹ״̬
                                            
/* Starting sampling rate.����Ƶ�� */       
//ԭ��Ϊ100

#define DEFAULT_MPU_HZ  (200)               	//�趨MPU�Ĳ����ʣ�DMP�����Ƶ�ʣ�ע�⣺��DMP��ʱ������ʹMPU�Ĳ����ʺ�DMP�Ĳ����ʱ���һ�£�

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
	

//���յ����ݣ����ڸ����ⲿ�����ָ�������MPU����Ӧ�Ĺ��ܣ�
struct rx_s {
    unsigned char header[3];		//���ⲿ���յ����ݰ�
    unsigned char cmd;      		//����ָ��
};

//Ӳ�������ӿڣ���־�㣩����������ȡֵ��Χ�μ�inv_mpu.c�е�chip_cfg_s�ṹ���е�ע�ͣ�
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};   //��ʼ��hal��ȫ����Ա����λΪ0

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;   //����ָʾ��û�н��յ��ⲿ�µ�����

/*�������
 The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};


																					 
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

///*�����趨������/�رմ�����
// Handle sensor on/off combinations. */
//static void setup_gyro(void)
//{
//    unsigned char mask = 0;
//    if (hal.sensors & ACCEL_ON)
//        mask |= INV_XYZ_ACCEL;
//    if (hal.sensors & GYRO_ON)
//        mask |= INV_XYZ_GYRO;
//    /* If you need a power transition, this function should be called with a
//     * mask of the sensors still enabled. The driver turns off any sensors
//     * excluded from this mask.
//     */
//    mpu_set_sensors(mask);
//    if (!hal.dmp_on)
//        mpu_configure_fifo(mask);
//}

//���ƻص�����
//static void tap_cb(unsigned char direction, unsigned char count)
//{
//    char data[2];
//    data[0] = (char)direction;
//    data[1] = (char)count;
//    send_packet(PACKET_TYPE_TAP, data);
//}


//��׿�豸Ҫ�����ʾ����Ļص�����
//static void android_orient_cb(unsigned char orientation)
//{
//    send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
//}



/**************************************************************************
*�Լ죬��У��ƫ��
ע�⣺�������Ǽ����ٶȵģ��������������Լ��У׼ʱ�����뱣�־�ֹ��ͬʱDMP��
һ���ʹ�����������������й����о�ֹ8�����ϵ�ʱ��ͻ��Զ�����У׼�����ǣ�
���������ǵ��ۼ����
�����ټ�ʱ�����ٶȵģ����Լ��ټ����Լ��У׼��ʱ��Z��������������һ�£�
��������ļ��ټƵ�ֵ�����Լ�ʱZ��ķ�����Ϊ��׼������һ�㣬Ϊ�˸�����
��ʹ��MPU��һ�㲻���м��ټƵ�У׼��ֱ��ʹ���ټ���Ĭ�ϵķ��򣨼���оƬ�����
����ϵ�е�Z�ᣩ��Ϊ��׼�����ǣ���Ҫʹ�÷�ˮƽ����ΪMPU�ĳ�ʼ���Ŀ��ʱ����
������м��ټƵ�У׼
**************************************************************************/
void run_self_test(void)
{
    int result;
    //char test_packet[4] = {0};
    long gyro[3], accel[3];

		//�������Լ죬������ƫ��ֵ
    result = mpu_run_self_test(gyro, accel);
	
		//ע�⣬��������ֻ�����������Ǻͼ��ټƣ�����resultΪ0x03,�������������ǣ����ټƺʹ�����ʱ��resultΪ0x07
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);						//��������ǵ�����������
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);						//����У׼������
        mpu_get_accel_sens(&accel_sens);		//��ü��ټƵ�����������
			accel_sens = 0;						//д����䣬��У׼���ټƣ�ע�⣺У׼���ټ�ʱ���ᵼ�¼��ټ����ֵȡ�����豸��ʼ�Ƕȣ�����Ϊ���ټ�У׼����Ҫ��Z�����������һ�£��⵼����Щ�豸��ʼ��ʱ�򲻷���
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);					//����У׼���ټ�
    }

}

//�����ⲿ�����ָ������MPU��DMP
//static void handle_input(void)
//{
//    char c;
//    const unsigned char header[3] = "inv";
//    unsigned long pedo_packet[2];

//    /* Read incoming byte and check for header.
//     * Technically, the MSP430 USB stack can handle more than one byte at a
//     * time. This example allows for easily switching to UART if porting to a
//     * different microcontroller.
//     */
//    rx_new = 0;
//    cdcReceiveDataInBuffer((BYTE*)&c, 1, CDC0_INTFNUM);
//    if (hal.rx.header[0] == header[0]) {
//        if (hal.rx.header[1] == header[1]) {
//            if (hal.rx.header[2] == header[2]) {
//                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
//                hal.rx.cmd = c;
//            } else if (c == header[2])
//                hal.rx.header[2] = c;
//            else
//                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
//        } else if (c == header[1])
//            hal.rx.header[1] = c;
//        else
//            memset(&hal.rx.header, 0, sizeof(hal.rx.header));
//    } else if (c == header[0])
//        hal.rx.header[0] = header[0];
//    if (!hal.rx.cmd)
//        return;

//    switch (hal.rx.cmd) {
//    /* These commands turn the hardware sensors on/off. */
//    case '8':
//        if (!hal.dmp_on) {
//            /* Accel and gyro need to be on for the DMP features to work
//             * properly.
//             */
//            hal.sensors ^= ACCEL_ON;
//            setup_gyro();
//        }
//        break;
//    case '9':
//        if (!hal.dmp_on) {
//            hal.sensors ^= GYRO_ON;
//            setup_gyro();
//        }
//        break;
//    /* The commands start/stop sending data to the client. */
//    case 'a':
//        hal.report ^= PRINT_ACCEL;
//        break;
//    case 'g':
//        hal.report ^= PRINT_GYRO;
//        break;
//    case 'q':
//        hal.report ^= PRINT_QUAT;
//        break;
//    /* The hardware self test can be run without any interaction with the
//     * MPL since it's completely localized in the gyro driver. Logging is
//     * assumed to be enabled; otherwise, a couple LEDs could probably be used
//     * here to display the test results.
//     */
//    case 't':
//        run_self_test();
//        break;
//    /* Depending on your application, sensor data may be needed at a faster or
//     * slower rate. These commands can speed up or slow down the rate at which
//     * the sensor data is pushed to the MPL.
//     *
//     * In this example, the compass rate is never changed.
//     */
//    case '1':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(10);
//        else
//            mpu_set_sample_rate(10);
//        break;
//    case '2':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(20);
//        else
//            mpu_set_sample_rate(20);
//        break;
//    case '3':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(40);
//        else
//            mpu_set_sample_rate(40);
//        break;
//    case '4':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(50);
//        else
//            mpu_set_sample_rate(50);
//        break;
//    case '5':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(100);
//        else
//            mpu_set_sample_rate(100);
//        break;
//    case '6':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(200);
//        else
//            mpu_set_sample_rate(200);
//        break;
//	case ',':
//        /* Set hardware to interrupt on gesture event only. This feature is
//         * useful for keeping the MCU asleep until the DMP detects as a tap or
//         * orientation event.
//         */
//        dmp_set_interrupt_mode(DMP_INT_GESTURE);
//        break;
//    case '.':
//        /* Set hardware to interrupt periodically. */
//        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
//        break;
//    case '7':
//        /* Reset pedometer. */
//        dmp_set_pedometer_step_count(0);
//        dmp_set_pedometer_walk_time(0);
//        break;
//    case 'f':
//        /* Toggle DMP. */
//        if (hal.dmp_on) {
//            unsigned short dmp_rate;
//            hal.dmp_on = 0;
//            mpu_set_dmp_state(0);
//            /* Restore FIFO settings. */
//            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//            /* When the DMP is used, the hardware sampling rate is fixed at
//             * 200Hz, and the DMP is configured to downsample the FIFO output
//             * using the function dmp_set_fifo_rate. However, when the DMP is
//             * turned off, the sampling rate remains at 200Hz. This could be
//             * handled in inv_mpu.c, but it would need to know that
//             * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
//             * put the extra logic in the application layer.
//             */
//            dmp_get_fifo_rate(&dmp_rate);
//            mpu_set_sample_rate(dmp_rate);
//        } else {
//            unsigned short sample_rate;
//            hal.dmp_on = 1;
//            /* Both gyro and accel must be on. */
//            hal.sensors |= ACCEL_ON | GYRO_ON;
//            mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//            /* Preserve current FIFO rate. */
//            mpu_get_sample_rate(&sample_rate);
//            dmp_set_fifo_rate(sample_rate);
//            mpu_set_dmp_state(1);
//        }
//        break;
//    case 'm':
//        /* Test the motion interrupt hardware feature. */
//        hal.motion_int_mode = 1;
//        break;
//    case 'p':
//        /* Read current pedometer count. */
//        dmp_get_pedometer_step_count(pedo_packet);
//        dmp_get_pedometer_walk_time(pedo_packet + 1);
//        send_packet(PACKET_TYPE_PEDO, pedo_packet);
//        break;
//    case 'x':
//        msp430_reset();
//        break;
//    case 'v':
//        /* Toggle LP quaternion.
//         * The DMP features can be enabled/disabled at runtime. Use this same
//         * approach for other features.
//         */
//        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
//        dmp_enable_feature(hal.dmp_features);
//        break;
//    default:
//        break;
//    }
//    hal.rx.cmd = 0;
//}


 /************************************************************************ 
������������׼����,����DMP�жϴ���ʱ����INT����������stm32���ŵ��ⲿ�ж�
������������������DMP�����Ѿ�׼���õ��ź�
Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 ********************************************************************/
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
 


//����ϵͳ 
void SYSTEM_Reset(void)
{
	__set_FAULTMASK(1);//���������ж�
	NVIC_SystemReset();//ϵͳ�ָ�
}


/***********************************************************
MPU��DMP��ʼ����������Լ��У׼
����ֵ��0:��ʼ���ɹ�
		����ֵ����ʼ��ʧ�ܣ������Ը��ݷ���ֵ�ж����������
***********************************************************/
u8 mpu_dmp_init(void)
{
		int result;

		//MPU��ʼ������λ�Ĵ�����ֵ������һЩ�Ĵ������ó�ʼֵ
    result = mpu_init();
		//mpu��ʼ��ʧ�ܣ�����ϵͳ  
		if (result)
		{
			SYSTEM_Reset();
			return 1;
		}
        
    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))//�������еĴ�����
			return 2;
		/* Push both gyro and accel data into the FIFO. */
    if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))//����FIFO�������������ݺͼ��ټƵ�ԭʼ����                                                 
			return 3;                                         
    if(mpu_set_sample_rate(DEFAULT_MPU_HZ))            //����MPU�����ʣ�����Ϊ100Hz����DMPû��ʹ��ʱ����MPU�Ĳ������ɴ����þ�������ʹ����DMPʱ������DMP�����Ƶ�ʾ��������ʣ�
			return 4;
		
    /* Read back configuration in case it was set improperly. */
//    mpu_get_sample_rate(&gyro_rate);
//    mpu_get_gyro_fsr(&gyro_fsr);
//    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;       //���������Ǻͼ��ټ��Ѵ򿪵�������Ϣ
//    hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    if(dmp_load_motion_driver_firmware())
			return 5;
    if(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
			return 6;
//    dmp_register_tap_cb(tap_cb);
//    dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |DMP_FEATURE_GYRO_CAL;
		if(dmp_enable_feature(hal.dmp_features))
			return 7;
    if(dmp_set_fifo_rate(DEFAULT_MPU_HZ))
			return 8;
		//DMP��ʼ���ɹ�
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

		//��ʼ�Լ죬�����µ���
		run_self_test();
	
		return 0;
//    __enable_interrupt();

//    /* Wait for enumeration. */
//    while (USB_connectionState() != ST_ENUM_ACTIVE);
}







/* Set up MSP430 peripherals. */
//static inline void platform_init(void)
//{
//	WDTCTL = WDTPW | WDTHOLD;
//    SetVCore(2);
//    msp430_clock_init(12000000L, 2);
//    if (USB_init() != kUSB_succeed)
//        msp430_reset();
//    msp430_i2c_enable();
//    msp430_int_init();

//    USB_setEnabledEvents(kUSB_allUsbEvents);
//    if (USB_connectionInfo() & kUSB_vbusPresent){
//        if (USB_enable() == kUSB_succeed){
//            USB_reset();
//            USB_connect();
//        } else
//            msp430_reset();
//    }
//}





unsigned long sensor_timestamp;				//ʱ���
short gyro[3], accel[3], sensors;			//gyro[3], accel[3]ԭʼ���ݱ�����,sensorsָʾ��FIFO�ж�ȡ��������ʲô���ݣ���Ԫ���������ǣ����ټƵȣ�
unsigned char more;										//ָʾFIFO���Ƿ���ʣ�������
long quat[4];																//��Ԫ��
#define q30 1073741824.0f                   //q30
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;			//��Ԫ������q30�����ʱ����
float pitch,roll,yaw;												//��̬��


//��ȡ���ݣ����ⲿ���õ�
u8 dmp_getdata(void)
{
	//�ж���λ���Ƿ������µ�����
//		if (rx_new)
//				/* A byte has been received via USB. See handle_input for a list of
//				 * valid commands.
//				 */
//				handle_input();
//		msp430_get_clock_ms(&timestamp);
	//���˶��ж�ģʽ����ʱ
//		if (hal.motion_int_mode) {
//				/* Enable motion interrupt. */
//				mpu_lp_motion_interrupt(500, 1, 5);		//�͹����˶��ж�����
//				hal.new_gyro = 0;                   	//�Ƚ�����ָʾ�����������ݵı�־������λ���Ա�ȴ�INT�ж�
//				/* Wait for the MPU interrupt. */
//				while (!hal.new_gyro)
//						__bis_SR_register(LPM0_bits + GIE);
//				/* Restore the previous sensor configuration. */
//				mpu_lp_motion_interrupt(0, 0, 0);
//				hal.motion_int_mode = 0;
//		}
	//��û�������ݱ���������DMPת������ʱ
//		if (!hal.sensors || !hal.new_gyro) {
//				/* Put the MSP430 to sleep until a timer interrupt or data ready
//				 * interrupt is detected.
//				 */
//				__bis_SR_register(LPM0_bits + GIE);
//				continue;
//		}
//ʹ����DMP���ܣ����Ի��ԭʼ���ݺ���Ԫ������
		if (hal.new_gyro && hal.dmp_on) {

				/* This function gets new data from the FIFO when the DMP is in
				 * use. The FIFO can contain any combination of gyro, accel,
				 * quaternion, and gesture data. The sensors parameter tells the
				 * caller which data fields were actually populated with new data.
				 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
				 * the FIFO isn't being filled with accel data.
				 * The driver parses the gesture data to determine if a gesture
				 * event has occurred; on an event, the application will be notified
				 * via a callback (assuming that a callback function was properly
				 * registered). The more parameter is non-zero if there are
				 * leftover packets in the FIFO.
				 */
				if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))//��ʼ��FIFO�ж�ȡ����
				{
					return 1;
				}
				//��FIFO���Ѿ�û��ʣ������ʱ,��ָʾʣ����������more��־������λ
				if (!more)
						hal.new_gyro = 0;//��ָʾ�������ݵı�����λ
				/* Gyro and accel data are written to the FIFO by the DMP in chip
				 * frame and hardware units. This behavior is convenient because it
				 * keeps the gyro and accel outputs of dmp_read_fifo and
				 * mpu_read_fifo consistent.
					*�����������Ǻͼ��ټƵ�ԭʼ���ݶ�����оƬ����Ŀ������ϵΪ��׼���������ģ��Ӷ�ʹ��
					*dmp_read_fifo������mpu_read_fifo������ȡ�������ݶ���һ�µģ�����ע�⣺��DMPû����ʱ��ʹ��
					*mpu_read_fifo����������������DMP��ʱ����ʹ��dmp_read_fifo��������ȡFIFO�е�����
				 */
//				if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)//��ȡ�����ǵ�ԭʼ���ݲ��ϱ����ݸ���λ��
//						send_packet(PACKET_TYPE_GYRO, gyro);
//				if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)//��ȡ���ټƵ�ԭʼ���ݲ��ϱ����ݸ���λ��
//						send_packet(PACKET_TYPE_ACCEL, accel);
				
				//��ԭʼ�����ϱ�����λ��
				//if(sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO   &&   sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL){
			//		ANO_DT_Send_Senser(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],0,0,0,0);
			//	}
				
				/* Unlike gyro and accel, quaternions are written to the FIFO in
				 * the body frame, q30. The orientation is set by the scalar passed
				 * to dmp_set_orientation during initialization.
				 ��Ԫ�������豸��װ������ϵ�����������Ϊ��׼��������ݵ�
				 */
//				if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
//						send_packet(PACKET_TYPE_QUAT, quat);
					if(sensors&INV_WXYZ_QUAT)///��ȡ��Ԫ�������ݣ������ݹ�ʽת������̬��
					{
						q0=quat[0]/q30;
						q1=quat[1]/q30;
						q2=quat[2]/q30;
						q3=quat[3]/q30;
						
						pitch=asin(-2*q1*q3+2*q0*q2)*57.3;
						roll=atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3;
						yaw=atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;
						
						//����̬�������ϱ�����λ��
				//		if(hal.report & PRINT_QUAT)
				//			ANO_DT_Send_Status(rol,pit,yaw,0,0,0);
					}
					return 0;
		}
		//δʹ��DMP������ȡԭʼ����
		else if (hal.new_gyro) 
		{
				unsigned char sensors, more;
				/* This function gets new data from the FIFO. The FIFO can contain
				 * gyro, accel, both, or neither. The sensors parameter tells the
				 * caller which data fields were actually populated with new data.
				 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
				 * being filled with accel data. The more parameter is non-zero if
				 * there are leftover packets in the FIFO.
				 */
				if(mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more)){				//����FIFO�л�ȡԭʼ����
					//���ݶ�ȡʧ��
					return 1;
				}
				if (!more)
					hal.new_gyro = 0;
		//		if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
		//			send_packet(PACKET_TYPE_GYRO, gyro);
		//		if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
		//			send_packet(PACKET_TYPE_ACCEL, accel);
				
				//��ԭʼ�����ϱ�����λ��
			//	if(sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO   &&   sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL){
			//		ANO_DT_Send_Senser(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],0,0,0,0);
			//	}
				return 0;
		}
		return 1;//��ȡʧ��
}



