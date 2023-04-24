#ifndef __USRAT2_H
#define __USRAT2_H 
#include "sys.h"	  	

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define USART2_MAX_TX_LEN 200
#define USART2_MAX_RX_LEN 200
extern u8 u1rxbuf[USART2_MAX_RX_LEN];						
extern u8 USART2_RX_FLAG;			
extern u8 USART2_RX_LEN;	

void DMA_USART2_Tx_Data(u8 *buffer, u32 size);
void Initial_UART2(unsigned long baudrate);
void DMA1_USART2_Init(void);

#endif

