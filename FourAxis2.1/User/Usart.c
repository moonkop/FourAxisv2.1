#include "Usart.h"


void USART1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA  , ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
}

void USART1_Config(void)
{
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
    USART1_GPIO_Config();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1,&USART_InitStructure);   
   
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE );
    
    USART_Cmd(USART1,ENABLE);
    
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}




u8 recstatu;				//表示是否处于一个正在接收的状态
u8 ccnt;             //计数
u8 packerflag;      //是否接收到一个完整的数据包标志
u8 rxbuf[RX_WIDTH]; //缓冲区

void USART2_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD  , ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
}

void USART2_Config(void)
{
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
    USART2_GPIO_Config();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART2,&USART_InitStructure);   
   
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );
    
    USART_Cmd(USART2,ENABLE);
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
}





void USART4_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC  , ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
}

void USART4_Config(void)
{
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
    USART4_GPIO_Config();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(UART4,&USART_InitStructure);   
   
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE );
    
    USART_Cmd(USART2,ENABLE);
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
}




//发送一字节数据给匿名上位机
void USART1_SendByByter(u8 Data)
{
	USART_GetFlagStatus(USART1, USART_FLAG_TXE);	
	USART_SendData(USART1, Data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	
}

//发送数组给匿名上位机
void Usart1_Send(u8 *data_to_send, u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
		USART1_SendByByter(data_to_send[i]);
	}
}

int fputc(int ch,FILE *f) 
{ 
   USART_SendData(USART1, (u8) ch);  
   while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
   return ch; 
} 


//发送一字节数据给遥控器
void USART2_SendByByter(u8 Data)
{
	USART_GetFlagStatus(USART2, USART_FLAG_TXE);	
	USART_SendData(USART2, Data);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
}

//发送数组给遥控器
void Usart2_Send(u8 *data_to_send, u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
		USART2_SendByByter(data_to_send[i]);
	}
}



