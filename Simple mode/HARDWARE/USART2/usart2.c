#include "usart2.h"
u8 Fore,Back,Left,Right;

u8 USART2_TX_BUF[USART2_MAX_TX_LEN]; 	
u8 u1rxbuf[USART2_MAX_RX_LEN];		
u8 u2rxbuf[USART2_MAX_RX_LEN];			
u8 witchbuf=0;                  		
u8 USART2_TX_FLAG=0;					
u8 USART2_RX_FLAG=0;			
u8 USART2_RX_LEN = 0;	
	
/**
 *@brief DMA和串口配置
**/
void DMA1_USART2_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						

	//DMA通道6寄存器设置为缺省
	DMA_DeInit(DMA1_Channel6);
	//
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);	
	//设置接收缓冲区首地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)u1rxbuf;  
	//数据传输方向，外设到内存
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;		
	//DMA通道缓存大小
	DMA1_Init.DMA_BufferSize = USART2_MAX_RX_LEN;	
	//外设地址不变
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	
	//内存地址递增
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;					
	//数据宽度8位
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		
	//数据宽度也是8位，内存
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		
	//正常模式
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;	
	//通道优先级为高
	DMA1_Init.DMA_Priority = DMA_Priority_High; 	
	//不设置为内存到内存传输
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;								
	//初始化
	DMA_Init(DMA1_Channel6,&DMA1_Init);

	DMA_DeInit(DMA1_Channel7);											
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);					
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART2_TX_BUF;              		
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								
	DMA1_Init.DMA_BufferSize = USART2_MAX_TX_LEN;							
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									
	DMA_Init(DMA1_Channel7,&DMA1_Init); 									

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						
	NVIC_Init(&NVIC_InitStructure);											

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
	NVIC_Init(&NVIC_InitStructure);										

	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);							
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);							

	DMA_Cmd(DMA1_Channel6,ENABLE);           								
	DMA_Cmd(DMA1_Channel7,DISABLE);           								

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);        					
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);        					
}

/**
 *@brief 串口2初始化函数
 *@param 波特率
**/
void Initial_UART2(unsigned long baudrate)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;										
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								
	GPIO_Init(GPIOA, &GPIO_InitStructure);										
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;										
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						
	GPIO_Init(GPIOA, &GPIO_InitStructure);										

	USART_InitStructure.USART_BaudRate = baudrate;									
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						
	USART_InitStructure.USART_StopBits = USART_StopBits_1;						
	USART_InitStructure.USART_Parity = USART_Parity_No ;						
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					
	USART_Init(USART2, &USART_InitStructure); 										
	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);									
	USART_ClearFlag(USART2,USART_FLAG_TC);										
	
	USART_Cmd(USART2, ENABLE);													
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;								
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;					
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;								
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
	NVIC_Init(&NVIC_InitStructure);
	
	DMA1_USART2_Init();															
}


/**
 *@brief DMA 发送函数
 *@param 数据地址
 *@param 长度
**/
void DMA_USART2_Tx_Data(u8 *buffer, u32 size)
{
	while(USART2_TX_FLAG);						
	USART2_TX_FLAG=1;							
	DMA1_Channel7->CMAR  = (uint32_t)buffer;	
	DMA1_Channel7->CNDTR = size;    			
	DMA_Cmd(DMA1_Channel7, ENABLE);				
}

/**
 *@brief DMA1发送完成中断
**/
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC7)!= RESET)	
	{
		DMA_ClearITPendingBit(DMA1_IT_TC7); 	
		USART_ClearFlag(USART2,USART_FLAG_TC);
		DMA_Cmd(DMA1_Channel7, DISABLE );   	
		USART2_TX_FLAG=0;					
	}
}

/**
 *@brief 串口2空闲中断
**/
void USART2_IRQHandler(void)                	
{
	//u8 *p;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)	
	{
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2,USART_FLAG_TC);		
		DMA_Cmd(DMA1_Channel6, DISABLE );   						
		USART2_RX_LEN = USART2_MAX_RX_LEN - DMA1_Channel6->CNDTR;	
		//p = u1rxbuf;
		USART2_RX_FLAG=1;		
		DMA1_Channel6->CNDTR = USART2_MAX_RX_LEN;					
		DMA_Cmd(DMA1_Channel6, ENABLE);     						
  }
}





