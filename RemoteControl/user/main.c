/***************************************************************************************
									声明
本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他不可
估量的BUG，修远智控不负任何责任。请勿商用！

程序版本号：	2.0
日期：			2017-1-1
作者：			东方萧雨
版权所有：		修远智控N0.1实验室
****************************************************************************************/
#include "stm32f10x.h"
#include "led.h"
#include "systick.h"
#include "usart.h"
#include "spi.h"
#include "nRF.h"
#include "nvic.h"
#include "adc_dma.h"
#include "tim_octigr.h"
#include "button.h"

int main(void)
{
	USART_Config();
	printf("usart is ready\r\n");
	SysTick_Init();
	NVIC_Config();
	BUTTON_Config();
	LED_Config();
//	LED_On(LED1|LED2);
	NRF_Config();
	TIM_OCTigrConfig();
	
	ADC_DmaConfig();	//这个必须最后一个配置，因为一旦配置好这个，ADC就会开始工作了，则DMA会开始每个一段时间产生中断，或者先关闭总中断，最后所有都配置完毕后在打开总中断
	
	while(1){}
}







