#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define C1 PBin(4)
#define C2 PBin(3)
#define C3 PAin(5)
#define C4 PAin(4)

#define SPEED_Y 40 //����(ǰ��)����趨�ٶ�
#define SPEED_Z 100//ƫ��(����)����趨�ٶ� 
#define ACCX_FIX 100
#define ACCY_FIX -200
#define ACCZ_FIX 16000
#define PI 3.14159265
/**/
#define WARN_MSG 1 //warn
#define SPD_MSG  2 //speed
#define POS_MSG  3 //position
#define PARA_MSG 4 // param
#define DES_MSG  5 //destination
#define ACK_MSG  6
#define INFO_MSG 7

#define SPD_MODE 0
#define POS_MODE 1


void EXTI9_5_IRQHandler(void);
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right,int target);
int Turn_UP(int gyro_Z, int RC);
int Yaw_control(int gyro_Z, int encoder_left, int encoder_right);

float anya_position(void);
float anya_velocity(void);
int anya_balance(void);
int anya_yaw(void);
int anya_omiga(void);
void LQR(void);
void Observer(void);

void Tracking(void);
void print(void);
void data_receive(void);
void data_receive2(void);
void data_receive3(void);
void sendmsg(void);
void state_info(void);
void float_limit(float *x, float range);
#endif
