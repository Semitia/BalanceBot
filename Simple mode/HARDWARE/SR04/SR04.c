#include "SR04.h"

/**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
 微信公众号【大鱼机器人】
 后台回复【平衡小车】：获取平衡小车全套DIY资料
 后台回复【电子开发工具】：获取电子工程师必备开发工具
 后台回复【电子设计资料】：获取电子设计资料包
 知乎：张巧龙 
**************************************************************************/
#define	TRIG_PIN       GPIO_Pin_10   //TRIG       
#define	ECHO_PIN       GPIO_Pin_11		//ECHO   

/*
 * 函数名：UltrasonicWave_Configuration
 * 描述  ：超声波模块的初始化
 * 输入  ：无
 * 输出  ：无	
 */
void SR04_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;					 //PB10接TRIG
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		     //设为推挽输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(GPIOB, &GPIO_InitStructure);	                 //初始化外设GPIO 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				     //PB11接ECH0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		 //设为输入
  GPIO_Init(GPIOB,&GPIO_InitStructure);						 //初始化GPIOB
	
	 //GPIOB.11	  中断线以及中断初始化配置
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11);

 	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
}


void EXTI15_10_IRQHandler(void)
{
	delay_us(10);		                      //延时10us
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		
			TIM_SetCounter(TIM4,0);																		
			TIM_Cmd(TIM4, ENABLE);                               				//开启时钟
			while(GPIO_ReadInputDataBit(GPIOB,ECHO_PIN));	       				//等待低电平
			TIM_Cmd(TIM4, DISABLE);			                         				//定时器4失能
			SR04_Distance=TIM_GetCounter(TIM4)*1.7;						//计算距离&&SR04<150
			//printf("%.2f\r\n",SR04_Distance);
			EXTI_ClearITPendingBit(EXTI_Line11);  //清除EXTI2线路挂起位
	}		

}

/*
 * 函数名：UltrasonicWave_StartMeasure
 * 描述  ：开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
 * 输入  ：无
 * 输出  ：无	
 */
void SR04_StartMeasure(void)
{
  GPIO_SetBits(GPIOB,TRIG_PIN); 		  //送>10US的高电平RIG_PORT,TRIG_PIN这两个在define中有说
  delay_us(20);		                      //延时20US
  GPIO_ResetBits(GPIOB,TRIG_PIN);
}

void Timer4_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位		 
}
void TIM4_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		}
}
/******************* (C) 1209 Lab *****END OF FILE************/
