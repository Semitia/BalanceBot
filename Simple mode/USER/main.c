#include "sys.h"
/****************************全局变量*************************************/  
state_t past,next,now;

float Voltage;  														 //电池电压采样相关的变量
float pitch,roll,yaw; 								  					 //欧拉角(姿态角)
float aacx,aacy,aacz;													 //加速度传感器原始数据
short gyrox,gyroy,gyroz;											 	 //陀螺仪原始数据

float SR04_Distance;                 									 //超声波测距

int   Encoder_Left,Encoder_Right;         		 //左右编码器的脉冲计数
int 	Moto1=0,Moto2=0;												 //计算出来的最终赋给电机的PWM

int Velocity=0,Turn=0;
u8 Mode=0; //0静止，1速度控制，2位置控制，3位置控制（避障）
int Uart_Receive=0;
u8 key=0;								 									 //按键的键值
u8 TkSensor=0;
/***********************************************************************/
u8 buffer[USART2_MAX_TX_LEN]="Oh, nice to meet you!!\r\n";
u32 buf_size;

int main(void)	
{ 
	LED_Init();                    //=====初始化与 LED 连接的IO
	PlugIn_Init();								 //=====初始化与 USB 连接的IO
	KEY_Init();                    //=====初始化与按键连接的IO
	delay_init();	    	           //=====延时函数初始化	
	uart1_init(115200);	           //=====串口1初始化		
	DMA1_USART2_Init( );
	uart3_init(9600);
	delay_ms(100);
	/*****************修改蓝牙的默认通信波特率以及蓝牙默认的名字******************/
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+NAMEAnya!\r\n");//发送蓝牙模块指令--设置名字为：Bliz
	delay_ms(100);	
	Uart3SendStr("AT+BAUD8\r\n"); 		 //发送蓝牙模块指令,将波特率设置成115200
	delay_ms(100);		
	uart3_init(115200);
/*****************************************************************************/	
	Adc_Init();                    //=====初始化ADC
	//SR04_Configuration();
	Encoder_Init_TIM2();           //=====初始化编码器2
	Encoder_Init_TIM3();
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
	MPU_Init();					    			 //=====初始化MPU6050
	mpu_dmp_init();								 //=====初始化MPU6050的DMP模式					 
	TIM1_PWM_Init(7199,0);   			 //=====初始化PWM 10KHZ,用于驱动电机。 
	//TIM1_PWM_Init(10000,7199);
	delay_ms(1000);								 //=====延时1s 解决小车上电轮子乱转的问题
	Motor_Init();									 //=====初始化与电机连接的硬件IO接口 
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms定时中断初始化
	NVIC_Configuration();					 //=====中断优先级分组,其中包含了所有的中断优先级的配置,方便管理和一次性修改。
	oled_first_show();					   //只需要显示一次的字符,在此刷新一次即可。
	Timer4_Init(5000,7199);	    	 //=====超声波定时器初始化
	//Initial_UART2(9600);
	//DMA1_USART2_Init( );
	
	
	while(1)	
	{
		static u8 cnt=0;
		oled_show();
		buf_size = sizeof(buffer);
		if(cnt>=20)
		{
			cnt=0;
			//sendmsg();
			//state_info();
		}
		LED = !LED;
		cnt++;
	}
}


