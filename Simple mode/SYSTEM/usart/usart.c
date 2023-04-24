#include "usart.h"

#define USART1_DMA 1

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

u8 USART_RX_BUF[USART_REC_LEN];     //????,??64???.
//????
//bit7,??????
//bit6,???0x0d
//bit5~0,??????????
u8 USART_RX_STA=0;       //??????

//#if USART1_DMA
u8 USART1_TX_BUF[USART2_MAX_TX_LEN]; 	
u8 usart1_rxbuf[USART2_MAX_RX_LEN];				           		
u8 USART1_TX_FLAG=0;					
u8 USART1_RX_FLAG=0;			
u8 USART1_RX_LEN = 0;	
	
	
/**
 *@brief DMA和串口配置
**/
void DMA1_USART1_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						

	//DMA通道6寄存器设置为缺省
	DMA_DeInit(DMA1_Channel5);
	//
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);	
	//设置接收缓冲区首地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)usart1_rxbuf;  
	//数据传输方向，外设到内存
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;		
	//DMA通道缓存大小
	DMA1_Init.DMA_BufferSize = USART1_MAX_RX_LEN;	
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
	DMA_Init(DMA1_Channel5,&DMA1_Init);

	DMA_DeInit(DMA1_Channel4);											
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);					
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART1_TX_BUF;              		
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								
	DMA1_Init.DMA_BufferSize = USART1_MAX_TX_LEN;							
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									
	DMA_Init(DMA1_Channel4,&DMA1_Init); 									

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						
	NVIC_Init(&NVIC_InitStructure);											

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
	NVIC_Init(&NVIC_InitStructure);										

	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);							
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);							

	DMA_Cmd(DMA1_Channel5,ENABLE);           								
	DMA_Cmd(DMA1_Channel4,DISABLE);           								

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);        					
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);        					
}

/**
 *@brief DMA 发送函数
 *@param 数据地址
 *@param 长度
**/
void DMA_USART1_Tx_Data(u8 *buffer, u32 size)//单位为字节
{
	if(!size) {return;}
	while(USART1_TX_FLAG);						
	USART1_TX_FLAG=1;							
	DMA1_Channel4->CMAR  = (uint32_t)buffer;	
	DMA1_Channel4->CNDTR = size;    			
	DMA_Cmd(DMA1_Channel4, ENABLE);				
}

char line[3] = "\r\n";
void printf_s(char *s)
{
	//char *p = s;
	uint32_t size = strlen(s);
/*if(newline)
	{
		u8 i;
		char *str = (char*)malloc(size+2);
		for(i=0; i<size; i++) 
		{str[i]=s[i];}
		strcat(str,"\r\n");
		size+=2;
		p = str;
	}*/
	DMA_USART1_Tx_Data(s,size);
	//if(newline) DMA_USART1_Tx_Data(line,2);
	/*
	while(USART1_TX_FLAG);						
	USART1_TX_FLAG=1;			
	DMA1_Channel4->CMAR  = (uint32_t)p;	
	DMA1_Channel4->CNDTR = size;    			
	DMA_Cmd(DMA1_Channel4, ENABLE);
	*/
//	if(newline) free(p);
	return;
}

#define UP 3
#define DOWN -2
void printf_f(char *name, float data)
{
	char *str = (char*)malloc(UP+DOWN+2);
	int i,p=0;//p指向str的第几位
	bool reach_flag=0;//遇到首位数字
	printf_s(name);
	if(data<0) {str[p++]='-';data*=-1;}
	for(i=UP;i>=0;i--)
	{
		char num = data/(pow(10,i));
		if(!num && !reach_flag) {continue;}
		data-=num*pow(10,i);
		str[p++] = num+48;
		reach_flag=1;
	}
	if(!reach_flag) {str[p++] = '0';}
	str[p++] = '.';
	for(i=-1;i>=DOWN;i--)
	{
		char num = data/(pow(10,i));
		data-=num*pow(10,i);
		str[p++] = num+48;
	}
	str[p] = '\0';
	for(i=0;i<p;i++)
	{
		DMA_USART1_Tx_Data(&str[i],1);
	}	
	//DMA_USART1_Tx_Data(str,p);
	//printf_s(str,0);
	free(str);
	return;
}

/*
	p 可以看做是保留的小数位数
*/
int f_to_u(float data, u8 p)
{
	int t = (int)floor(data*pow(10,p));
	
	/*DEBUG*/
	//u8 high= t>>8;
	//u8 low = t&0x00FF;
	//DMA_USART1_Tx_Data("DEBUG ",6);
	//DMA_USART1_Tx_Data(&high,1);
	//Uart3SendByte(high);
	//Uart3SendByte(low);	
	//DMA_USART1_Tx_Data(&low ,1);
	//DMA_USART1_Tx_Data("\r\n",2);
	return t;
}

void swrite(char *buf, int txt, u8 start)
{
	u8 i;
	if(txt<0) {buf[start]='-';txt*=-1;}
	else 			{buf[start]='+';}
	start++;
	for(i=0;i<4;i++)
	{
		u16 f=(u16)(pow(10,3-i));
		buf[start+i]=txt/f + 48;
		txt %= f;
	}
	//buf[++start]=0;
	return;
}


void printf1(u8 *name, float data)
{
	while(USART1_TX_FLAG);						
	USART1_TX_FLAG=1;							
	//DMA1_Channel4->CMAR  = (uint32_t)buffer;	
	//DMA1_Channel4->CNDTR = size;    			
	//DMA_Cmd(DMA1_Channel4, ENABLE);		
}

/**
 *@brief DMA1发送完成中断
**/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC4)!= RESET)	
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4); 	
		USART_ClearFlag(USART1,USART_FLAG_TC);
		DMA_Cmd(DMA1_Channel4, DISABLE );   	
		USART1_TX_FLAG=0;					
	}
}

/**
 *@brief 串口1空闲中断

void USART1_IRQHandler(void)                	
{
	//u8 *p;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)	
	{
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1,USART_FLAG_TC);		
		DMA_Cmd(DMA1_Channel5, DISABLE );   						
		USART1_RX_LEN = USART1_MAX_RX_LEN - DMA1_Channel5->CNDTR;	
		//p = u1rxbuf;
		USART1_RX_FLAG=1;		
		DMA1_Channel5->CNDTR = USART1_MAX_RX_LEN;					
		DMA_Cmd(DMA1_Channel5, ENABLE);     						
  }
}**/

//不开启DMA
int start_time,end_time;

void USART1_IRQHandler(void)                	//??1??????
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //????(?????????0x0d 0x0a??)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//????????
		if((USART_RX_STA&0x80)==0)//?????
		{
			if(USART_RX_STA&0x40)//????0x0d
			{
				if(Res!=0x0a) USART_RX_STA=0;//????,????
				else 
				{
					USART_RX_STA|=0x80;	//????? 
					end_time = TIM1->CNT;
				}
			}
			else //????0X0D
			{	
				if(Res==0x0d) USART_RX_STA|=0x40;
				else
				{
					if( (USART_RX_STA&0X3F) == 0) start_time = TIM1->CNT;
					USART_RX_BUF[USART_RX_STA&0X3F]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>63) USART_RX_STA=0;//??????,??????	  
				}		 
			}
		}   		 
   } 
} 



void uart1_init(u32 bound)
{
	//GPIO????
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//USART ?????
	USART_InitStructure.USART_BaudRate = bound;//?????9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//????
	USART_Cmd(USART1, ENABLE);                    //???? 

}

/*USART3*/
u8 USART3_RX_BUF[USART_REC_LEN];     //接收缓冲,最大64个字节.
//接收状态
//bit7，接收完成标志
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
u8 USART3_RX_STA=0;       //接收状态标记

void uart3_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//USART3_TX   PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//USART3_RX	  PB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART3, ENABLE);                    //使能串口 
}

void Uart3SendByte(char byte)   //串口发送一个字节
{
        USART_SendData(USART3, byte);        //通过库函数  发送数据
        while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
        //等待发送完成。   检测 USART_FLAG_TC 是否置1；    //见库函数 P359 介绍
}

void Uart3SendBuf(char *buf, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)Uart3SendByte(*buf++);
}

void Uart3SendStr(char *str)
{
	u16 i,len;
	len = strlen(str);
	for(i=0; i<len; i++)Uart3SendByte(*str++);
}

void USART3_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
		if((USART3_RX_STA&0x80)==0)//接收未完成
		{
			if(USART3_RX_STA&0x40)//接收到了0x0d
			{
				if(Res!=0x0a)USART3_RX_STA=0;//接收错误,重新开始
				else USART3_RX_STA|=0x80;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART3_RX_STA|=0x40;
				else
				{
					USART3_RX_BUF[USART3_RX_STA&0X3F]=Res ;
					USART3_RX_STA++;
					if(USART3_RX_STA>63)USART3_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
    } 
} 

int tr(char t)
{ 
    return t-48; 
}

/**
 * @brief 将字符串从start位开始的num位转换为float数
 * @param s 字符串
 * @param start 起始位
 * @param num 位数
 * @param p 首个数乘 10^p
 * @return float 
 */
float tr_s(u8 *s, int start, int num, int p)
{
	float ans=0;
	short negative=1;
	if(s[start++] == '-') {negative=-1;}
	num--;
	while(num>0)
	{
			ans+= tr(s[start++])*pow(10,p--);
			num--;
	}
	ans*=negative;
	return ans;
}

int tr_head(u8 *s, int start, int num, int p)
{
	int ans=0;
	while(num>0)
	{
		ans+= tr(s[start++])*pow(10,p--);
		num--;
	}
	return ans;
}
