 /**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
/****************************全局变量*************************************/    
float Voltage;  															 //电池电压采样相关的变量
float pitch,roll,yaw; 								  			 //欧拉角(姿态角)
short aacx,aacy,aacz;													 //加速度传感器原始数据
short gyrox,gyroy,gyroz;											 //陀螺仪原始数据
int   Encoder_Left,Encoder_Right;         		 //左右编码器的脉冲计数
int 	Moto1=0,Moto2=0;												 //计算出来的最终赋给电机的PWM

int Velocity=0,Turn=0;
u8 Mode=99; //98蓝牙 99红外循迹 100 PS2
int Uart_Receive=0;
u8 key=0;								 									 //按键的键值
u8 TkSensor=0;
/***********************************************************************/
int main(void)	
{ 
	LED_Init();                    //=====初始化与 LED 连接的IO
	USB_Init();										 //=====初始化与 USB 连接的IO
	KEY_Init();                    //=====初始化与按键连接的IO
	delay_init();	    	           //=====延时函数初始化	
	uart1_init(115200);	          	 //=====串口1初始化
	uart2_init(9600);							 //=====串口2初始化即蓝牙初始化
	delay_ms(100);
/*****************修改蓝牙的默认通信波特率以及蓝牙默认的名字******************
	Uart2SendStr("AT\r\n");
	Uart2SendStr("AT+NAME333DayuRobot\r\n");//发送蓝牙模块指令--设置名字为：Bliz
	delay_ms(100);	
	Uart2SendStr("AT+BAUD8\r\n"); 		 //发送蓝牙模块指令,将波特率设置成115200
	delay_ms(100);		
//	uart2_init(115200);
*****************************************************************************/	
	NVIC_Configuration();					 //=====中断优先级分组,其中包含了所有的中断优先级的配置,方便管理和一次性修改。
	Adc_Init();                    //=====初始化ADC
	Tracking_Init();
	Encoder_Init_TIM2();           //=====初始化编码器2
	Encoder_Init_TIM3();
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
	MPU_Init();					    			 //=====初始化MPU6050
	mpu_dmp_init();								 //=====初始化MPU6050的DMP模式					 
	TIM1_PWM_Init(7199,0);   			 //=====初始化PWM 10KHZ,用于驱动电机。 
	delay_ms(1000);								 //=====延时1s 解决小车上电轮子乱转的问题
	Motor_Init();									 //=====初始化与电机连接的硬件IO接口 
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms定时中断初始化
	oled_first_show();					   //只需要显示一次的字符,在此刷新一次即可。
  while(1)	
	{
		oled_show();
		delay_ms(50); //20HZ的显示频率，屏幕无需时刻刷新。
	}
}
void Tracking_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
} 
