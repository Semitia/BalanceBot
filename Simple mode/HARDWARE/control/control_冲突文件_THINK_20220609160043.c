#include "control.h"
#include "usart2.h"
/**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
				 在MPU6050的采样频率设置中，设置成100HZ，即可保证6050的数据是10ms更新一次。
				 读者可在imv_mpu.h文件第26行的宏定义进行修改(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_Kp;

float Mechanical_angle=0; 
float Target_Speed=0;	//期望速度（俯仰）。用于控制小车前进后退及其速度。
float Turn_Speed=0;		//期望速度（偏航）

//针对不同车型参数，在sys.h内设置define的电机类型
float balance_UP_KP=BLC_KP; 	 // 小车直立环PD参数
float balance_UP_KD=BLC_KD;

float velocity_KP=SPD_KP;     // 小车速度环PI参数
float velocity_KI=SPD_KI;

float Turn_Kd=TURN_KD;//转向环KP、KD
float Turn_KP=TURN_KP;

#define SPEED_Y 40 //俯仰(前后)最大设定速度
#define SPEED_Z 100//偏航(左右)最大设定速度 

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)
	{
		EXTI->PR=1<<5;                                           //===清除LINE5上的中断标志位   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===得到欧拉角（姿态角）的数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===得到陀螺仪数据
		Encoder_Left=Read_Encoder(2);                           //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		Encoder_Right=-Read_Encoder(3);                           //===读取编码器的值
		
		Tracking();
		
		/*前后*/
		switch(Mode)
		{
			case 98://蓝牙模式
				if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
				if(Fore==1)Target_Speed--;//前进1标志位拉高-->需要前进
				if(Back==1)Target_Speed++;//
				/*左右*/
				if((Left==0)&&(Right==0))Turn_Speed=0;
				if(Left==1)Turn_Speed-=30;	//左转
				if(Right==1)Turn_Speed+=30;	//右转
				/*转向约束*/
				if((Left==0)&&(Right==0))Turn_Kd=-0.6;//若无左右转向指令，则开启转向约束
				else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束
				break;
			case 99://循迹模式
				switch(TkSensor)
				{
					case 15:
						Target_Speed=0;
						Turn_Speed=0;
						break;
					case 9:
						Target_Speed--;
						Turn_Speed=0;
						break;
					case 2://向右转
						Target_Speed--;
						Turn_Speed=15;
						break;
					case 4://向左转
						Target_Speed--;
						Turn_Speed=-15;
						break;
					case 8:
						Target_Speed=-10;
						Turn_Speed=-80;
						break;
					case 1:
						Target_Speed=-10;
						Turn_Speed=80;
						break;
				}
				break;
		}
			
		Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
		Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//限幅( (20*100) * 100)
			
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===直立环PID控制	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,Target_Speed);       //===速度环PID控制	 
		Turn_Pwm =Turn_UP(gyroz,Turn_Speed);        //===转向环PID控制
		Moto1=Balance_Pwm-Velocity_Pwm+Turn_Pwm;  	            //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                 //===计算右轮电机最终PWM
	  Xianfu_Pwm();  																					 //===PWM限幅
		Turn_Off(pitch,12);																 //===检查角度以及电压是否正常
		Set_Pwm(Moto1,Moto2);                                    //===赋值给PWM寄存器  
	}
}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、机械平衡角度（机械中值）、角速度
返回  值：直立控制PWM
作    者：大鱼电子
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 //===求出平衡的角度中值和机械相关
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制
入口参数：电机编码器的值
返回  值：速度控制PWM
作    者：大鱼电子
**************************************************************************/
int velocity(int encoder_left,int encoder_right,int gyro_Z)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral;
   //=============速度PI控制器=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right);//-target;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-gyro_Z;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
		Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===速度控制	
	  if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===电机关闭后清除积分
	  return Velocity;
}
/**************************************************************************
函数功能：转向PD控制
入口参数：电机编码器的值、Z轴角速度
返回  值：转向控制PWM
作    者：大鱼电子
**************************************************************************/

int Turn_UP(int gyro_Z, int RC)
{
	int PWM_out;
	
	PWM_out=Turn_Kd*gyro_Z + Turn_KP*RC;
	return PWM_out;
}

void Tracking()
{
	TkSensor=0;
	TkSensor+=(C1<<3);
	TkSensor+=(C2<<2);
	TkSensor+=(C3<<1);
	TkSensor+=C4;
}
