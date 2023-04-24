#include "control.h"
/**************************************************************************
函数功能：所有的控制代码都在这里面
        5ms定时中断由MPU6050的INT引脚触发
        严格保证采样和数据处理的时间同步	
				在MPU6050的采样频率设置中，设置成100HZ，即可保证6050的数据是10ms更新一次。
				读者可在imv_mpu.h文件第26行的宏定义进行修改(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/

int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_Kp;
int oled1;
float oled_A,oled_B,oled_C,oled_D,oled_E,oled_F;
u8 motion_mode=0;
bool ACK=0;
u8 off_flag=0;

float Mechanical_angle=MECHI; 
float target_x=0, target_y=0;
//float target_yaw=0;
float target_speed=0;	//期望速度。用于控制小车前进后退及其速度。
float target_omiga=0; //期望角速度
float Turn_Speed=0;		//期望速度（偏航）
float Target_Yaw=0;
float target_angle = MECHI;
int target_position=0;

//针对不同车型参数，在sys.h内设置define的电机类型
float balance_UP_KP=BLC_KP; 	 // 小车直立环PD参数
float balance_UP_KD=BLC_KD;

float velocity_KP=SPD_KP;     // 小车速度环PI参数
float velocity_KI=SPD_KI;
float velocity_KD=SPD_KD;

float Turn_Kd=TURN_KD;//转向环KP、KD
float Turn_KP=TURN_KP;
float Turn_KI=TURN_KI;

float Position_KP=POSI_KP;

u8 SR04_Counter=0;
u8 Voltage_Counter=0;
u8 rec_data[18];
short get_aacx, get_aacy, get_aacz;
int n_cnt=0,tim_cnt[100]={0};
u8 i;

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)
	{
		EXTI->PR=1<<5;                                           //===清除LINE5上的中断标志位   
		state_update();
	
		Voltage_Counter++;
		if(Voltage_Counter>=200)									 //get the voltage of battery
		{
			Voltage_Counter=0;
			Voltage=Get_battery_volt();		                         //===读取电池电压
		}
		
		/*
		//target_speed = anya_position();
		target_angle = (float) anya_velocity() + MECHI;
		Balance_Pwm  = anya_balance();
		switch(Mode)
		{
			case 0:
			{
				Turn_Pwm = anya_yaw();
				break;
			}
			case 1:
			{
				Turn_Pwm = anya_omiga();
				break;
			}
			case 2:
			{
				Turn_Pwm = anya_yaw();
				break;
			}
		}
		//Turn_Pwm = anya_yaw();
		Moto1=Balance_Pwm+Turn_Pwm;  	            //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Turn_Pwm;                 //===计算右轮电机最终PWM
	    Xianfu_Pwm();  																					 //===PWM限幅
		Turn_Off(now.pitch,Voltage,off_flag);																 //===检查角度以及电压是否正常
		Set_Pwm(Moto1,Moto2);    //===赋值给PWM寄存器  
		oled_v = target_angle, oled_tar_w = target_omiga, oled_p = target_speed, oled_theta = now.theta;
		*/
		//LQR();
		Observer();
		Xianfu_Pwm(); 
		//Turn_Off(now.pitch,Voltage,off_flag);	
		Set_Pwm(Moto1,Moto2); 
		
		oled1=Moto1;
		
		data_receive();
		data_receive3();
	}
}

const float LQR_K[4] = {-9.33,-0.04,0,0};
float K_u_to_out=1000;//u为实际需求驱动力，但是实际控制电机的输出是output，中间乘上一个转变系数
void LQR(void)
{
	u8 i;
	float output=0;
	static float LQR_X[4];
	//state space: θ θ' x x'
	LQR_X[1]=(now.pitch_rad - LQR_X[0])/delta_t;
	LQR_X[0]=now.pitch_rad;
	LQR_X[3]=(now.x-LQR_X[2])/delta_t;
	LQR_X[2]=now.x;

	oled_A = LQR_X[0];
	oled_B = LQR_X[1];
	oled_C = LQR_X[2];
	oled_D = LQR_X[3];
	//
	for(i=0;i<4;i++) output-=LQR_K[i]*LQR_X[i]*K_u_to_out;
	Moto1 = output/2;
	Moto2 = output/2;
	oled_E = output;
	return;
}

const float L[3] = {6,477.5,-0.017186};
float delta_X[3];//theta,dot_theta,alpha
void Observer(void)
{
	u8 i;
	float output=0;
	static float X_hat[3];
	float y_err = now.pitch_rad -X_hat[0];
	delta_X[0] = X_hat[1] + L[0]*y_err;
	delta_X[1] = 465.5*(now.pitch_rad - X_hat[2])-(Moto1+Moto2)/K_u_to_out + L[1]*y_err;
	delta_X[2] = L[2]*y_err;
	for(i=0;i<3;i++) delta_X[i]*=delta_t;//这个很重要
	for(i=0;i<3;i++) X_hat[i]+=delta_X[i];
	
	float_limit(&X_hat[0],1);
	float_limit(&X_hat[1],10);
	float_limit(&X_hat[2],1);
	//printf("y_error %.2f\r\n",y_err);
	//printf("delta %.2f %.2f %.2f  observe %.2f %.2f %.2f\r\n",delta_X[0],delta_X[1],delta_X[2],X_hat[0],X_hat[1],X_hat[2]);
	
	oled_A = X_hat[0];
	oled_B = X_hat[1];
	oled_C = X_hat[2];
	oled_D = y_err;

	output = -LQR_K[0]*(X_hat[0]-X_hat[2])-LQR_K[1]*X_hat[1];
	output *= K_u_to_out;
	Moto1 = output/2;
	Moto2 = output/2;
	oled_E = output;
	return;
}

void int_limit(int *x, float range)
{
	if(*x>range)  {*x = range;}
	if(*x<-range) {*x = -range;}
	return;
}

void float_limit(float *x, float range)
{
	if(*x>range)  {*x = range;}
	if(*x<-range) {*x = -range;}
	return;
}

int anya_balance(void)
{
	/*
	float error;
	int output;
	error = now.pitch - target_angle;
	output = balance_UP_KP*error + balance_UP_KD*(now.gyroy);
	*/
	return balance_UP_KP*(now.pitch - target_angle) + balance_UP_KD*(now.gyroy);
}




int anya_omiga(void)
{
	int pwm_out;
	float error = target_omiga - now.w;
	static float error_sum = 0;
	error_sum+=error;
	pwm_out = error*Turn_KP + error_sum*Turn_KI;
	//oled_turn_pwm = (float)pwm_out;
	return pwm_out;
}


u8 warn=0;
u8 info_req=0;
const u8 for_bac=1, turn=2;
void data_receive(void)
{
	u8 *buf;
	u8 msg_type;

	if(!(USART_RX_STA&0x80))
	{return;}
	buf = USART_RX_BUF;

	
	msg_type = buf[0]-'0';
	switch(msg_type)
	{
		case WARN_MSG:
		{
			Mode = 3;
			warn = tr_s(buf,1,3,2);//100*(rx_buf[1]-'0') + 10*(rx_buf[2]-'0') + (rx_buf[3]-'0');
			//info_req = WARN_MSG;
			break;
		}
		
		case SPD_MSG:
		{
			u8 mov_mod = tr(buf[1]);
			Mode = 1;
			switch(mov_mod)
			{
				case for_bac:
				{
					target_speed = tr_s(buf,2,5,1);//(float)( 10*(USART3_RX_BUF[1]-'0') + (USART3_RX_BUF[2]-'0') + 0.1*(USART3_RX_BUF[3]-'0') );
					target_omiga = 0;
					break;
				}
				case turn:
				{
					target_speed = 0;
					target_omiga = tr_s(buf,2,5,1);
					break;
				}
			}
			//printf("speed:%.2f, ",target_speed);
			//info_req = SPD_MSG;
			break;
		}
		
		case POS_MSG://要前往的坐标
		{
			Mode = 2;
			target_x = tr_s(buf,1,4,2);//100*(rx_buf[1]-'0') + 10*(rx_buf[2]-'0') + (rx_buf[3]-'0') + 0.1*(rx_buf[4]-'0');
			target_y = tr_s(buf,5,4,2);//100*(rx_buf[5]-'0') + 10*(rx_buf[6]-'0') + (rx_buf[7]-'0') + 0.1*(rx_buf[8]-'0');
			Target_Yaw = tr_s(buf,9,4,2);
			//info_req = POS_MSG;
			break;
		}
		
		case PARA_MSG:
		{
			u8 para_type = tr(buf[1]);
			switch(para_type)
			{
				case 1:
				{
					K_u_to_out = tr_s(buf,3,6,4);
					printf("LQR_a = %.1f\r\n",K_u_to_out);
					break;
				}
				case 2:
				{
					break;
				}
				case 3:
				{
					break;
				}
				case 7:
				{
					//Turn_KP = tr_s(buf)
					break;
				}
				case 8:
				{
					break;
				}
			}
			break;
		}
		case ACK_MSG:
		{
			ACK=1;
			break;
		}
		
		
	}
	

	USART_RX_STA=0;

	return;
}

void data_receive2(void)
{
	//u8 ACK_u8[200],i;
	char ACK[200];
	//u32 ACK_size;
	if(!USART2_RX_FLAG)
	{return;}
	USART2_RX_FLAG = 0;
	
	balance_UP_KP = (float) ( 100*(u1rxbuf[0]-'0') + 10*(u1rxbuf[1]-'0') + (u1rxbuf[2]-'0') );
	sprintf(ACK, "UP_KP:%.0f, ",balance_UP_KP);
	balance_UP_KD = (float)( (u1rxbuf[4]-'0') + 0.1*(u1rxbuf[5]-'0') + 0.01*(u1rxbuf[6]-'0') );
	sprintf(ACK, "UP_KD:%.2f, ",balance_UP_KD);
	velocity_KP = (float)( 100*(u1rxbuf[8]-'0') + 10*(u1rxbuf[9]-'0') + (u1rxbuf[10]-'0') );
	sprintf(ACK, "V_KP:%.0f, ",velocity_KP);
	velocity_KI = (float)( (u1rxbuf[12]-'0') + 0.1*(u1rxbuf[13]-'0') + 0.01*(u1rxbuf[14]-'0') );
	sprintf(ACK, "V_KI:%.2f, ",velocity_KI);
	Mechanical_angle = (float)( (u1rxbuf[17]-'0') + 0.1*(u1rxbuf[18]-'0') + 0.01*(u1rxbuf[19]-'0') );
	if(u1rxbuf[16]-'0' == 1) {Mechanical_angle *= -1;}
	sprintf(ACK, "Mechanical:%.2f, ",Mechanical_angle);
	velocity_KD = (float)( 10*(u1rxbuf[21]-'0') + (u1rxbuf[22]-'0') + 0.1*(u1rxbuf[23]-'0') );
	sprintf(ACK, "V_KD:%.1f, ",velocity_KD);
	Turn_KP = (float)( (u1rxbuf[25]-'0') + 0.1*(u1rxbuf[26]-'0') + 0.01*(u1rxbuf[27]-'0') );
	sprintf(ACK, "Turn_KP:%.2f, ",Turn_KP);
	Turn_KI = (float)( (u1rxbuf[29]-'0') + 0.1*(u1rxbuf[30]-'0') + 0.01*(u1rxbuf[31]-'0') );
	sprintf(ACK, "Turn_KI:%.2f\r\n",Turn_KI);
	printf("%s\r\n",ACK);
	//ACK_size = sizeof(ACK);
	//for(i=0;i<ACK_size;i++) ACK_u8[i] = (u8)ACK[i];
	//DMA_USART2_Tx_Data(ACK_u8,ACK_size);
	return;
}

bool des_flag=0;
void data_receive3(void)
{
	//u8 len,t;
	if(USART3_RX_STA&0x80)
	{					   
		//len=USART3_RX_STA&0x3f;//得到此次接收到的数据长度
		int BTcmd= (int)tr_head(USART3_RX_BUF,0,3,2);
		switch(BTcmd)
		{
			case 666://destination
			{
				target_x = tr_s(USART3_RX_BUF,4,5,2);
				target_y = tr_s(USART3_RX_BUF,10,5,2);
				des_flag=1;
				break;
			}
			case 667://param
			{
				u8 para_type = tr(USART3_RX_BUF[4]);
				switch(para_type)
				{
					case 1:
					{
						break;
					}
					case 2:
					{
						break;
					}
					case 3:
					{
						break;
					}
					case 7:
					{
						Turn_KP = tr_s(USART3_RX_BUF,6,5,3);
						break;
					}
					case 8:
					{
						Turn_KI = tr_s(USART3_RX_BUF,6,5,2);
						break;
					}
				}
				break;
			}
			case 668://speed cmd
			{
				target_speed = tr_s(USART3_RX_BUF,4,5,1);
				//if(USART3_RX_BUF[3]-'0' == 1) {target_speed *= -1;}
				target_omiga = tr_s(USART3_RX_BUF,10,5,1);
				//if(USART3_RX_BUF[9]-'0' == 1) {target_omiga *= -1;}
				Mode=1;
				break;
			}
		}
		USART3_RX_STA=0;
	}
	return;
}
	
void sendmsg(void)
{
	u8 i;
	char buf[20];
	u8 len;
	if(!des_flag) 
	{
		sprintf(buf,"Waitting DES_CMD\r\n");
		len=18;
	}
	else if(!ACK)//DES_MSG & POS_MSG
	{
		int s1=f_to_u(target_x,1), s2=f_to_u(target_y,1), s3=f_to_u(now.x,1), s4=f_to_u(now.y,1), s5=f_to_u(now.theta,3);
		//sprintf(buf,"%u%d%d\r\n",DES_MSG,s1,s2);
		buf[0]=DES_MSG+48;
		swrite(buf,s1,1);
		swrite(buf,s2,6);
		swrite(buf,s3,11);
		swrite(buf,s4,16);
		swrite(buf,s5,21);
		len=27;
	}
	else//POS_MSG
	{
		int s1=f_to_u(now.x,1), s2=f_to_u(now.y,1), s3=f_to_u(now.theta,3);
		buf[0]=POS_MSG+48;
		swrite(buf,s1,1);
		swrite(buf,s2,6);
		swrite(buf,s3,11);
		//sprintf(buf,"%u%d%d%d\r\n",POS_MSG,s1,s2,s3);
		len=17;
	}
	for(i=0;i<len;i++)
	{
		DMA_USART1_Tx_Data(&buf[i],1);
	}
	//DMA_USART1_Tx_Data(buf,len);
	//printf_s(buf,0);
	
	switch(info_req)
	{
		case 0:
		{
			break;
		}
		case WARN_MSG:
		{
			char str[4];
			sprintf(str,"%u",warn);
			DMA_USART1_Tx_Data("INFO:WARN= ",12);
			DMA_USART1_Tx_Data(str,4);
			break;
		}
		case SPD_MSG:
		{
			if(target_speed>0) {DMA_USART1_Tx_Data("Forward",8);}
			else if(target_speed<0) {DMA_USART1_Tx_Data("Back",6);}
			else if(target_omiga<0) {DMA_USART1_Tx_Data("TurnRight",10);}
			else {DMA_USART1_Tx_Data("TurnLeft",9);}
			break;
		}
		case POS_MSG:
		{
			break;
		}
		case PARA_MSG:
		{
			break;
		}
		case DES_MSG:
		{
			break;
		}
	}
	
	return;
}

void state_info()
{
	u8 i;
	char buf[20];
	u8 len;

	int s1=f_to_u(now.x,1), s2=f_to_u(now.y,1), s4=f_to_u(now.w,1), s5=f_to_u(now.theta,1);
	buf[0]=POS_MSG+48;
	swrite(buf,s1,1);
	swrite(buf,s2,6);
	swrite(buf,s4,11);
	swrite(buf,s5,16);
	//sprintf(buf,"%u%d%d%d\r\n",POS_MSG,s1,s2,s3);
	len=22;
	for(i=0;i<len;i++)
	{
		DMA_USART1_Tx_Data(&buf[i],1);
	}
	return;
}

