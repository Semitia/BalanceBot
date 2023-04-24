#include "motion.h"
extern u8 warn;

void forward()
{

	return;
}

void reverse()
{

	return;
}

/**
 * @brief give the move command rely on car's position and target position
 * 1:前进
 * 2:顺时针转
 * 3:逆时针转
 * @param tx target x
 * @param ty target y
 * @return move command
 */
u8 plan(float tx,float ty)
{
	u8 i;
	bool wall[8];
	float tar_yaw = atan((ty-now.y)/(tx-now.x));//angle

	for(i=0;i<8;i++)
	{
		if((warn>>i)&0x01) 
		{ wall[i] = true; }
	}

	if (abs(tar_yaw - now.yaw)<YAW_THR) 
	{
		if(!wall[front])
		{return 1;}
		else if(!wall[left])
		{
			//更改now.move_cmd
			return 3;
		}
	}
	else if(tar_yaw < now.yaw) {return 2;}
	else {return 3;}
	return 0;//先随便写的
}

void state_update(void)
{
	float d_theta, d_x, d_y;

	mpu_dmp_get_data(&(now.pitch),&(now.roll),&(now.yaw));										 //===得到欧拉角（姿态角）的数据
	MPU_Get_Gyroscope(&(now.gyrox),&(now.gyroy),&(now.gyroz));
	now.v_left  = -Read_Encoder(2)*wheel;
	now.v_right =  Read_Encoder(3)*wheel;

	now.v = (float) (now.v_left + now.v_right)/2;
	now.w = (float) (now.v_right - now.v_left)/car_d;
	now.R = (float) (now.v) / (now.w);
	
	d_theta = past.v*delta_t/past.R;
	d_x = past.R*(1-cos(d_theta));
	d_y = past.R*sin(d_theta);
	now.x = past.x + d_x*sin(past.theta) + d_y*cos(past.theta);
	now.y = past.y - d_x*cos(past.theta) + d_y*sin(past.theta);
	now.theta = past.theta + d_theta;
	if(now.theta>PI)  now.theta-=PI;
	if(now.theta<-PI) now.theta+=PI;
	now.pitch_rad = now.pitch*PI/180;
	past = now;
	return;
}
