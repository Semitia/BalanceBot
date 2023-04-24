#ifndef __SYS_H
#define __SYS_H	 

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "led.h"
#include "oled.h"
#include "key.h"
#include "exti.h"
#include "pwm.h"
#include "USB_PlugIn.h"
#include "timer.h"
#include "adc.h"
#include "timer.h"
#include "motor.h"
#include "control.h"
#include "encoder.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "SR04.h"
#include "DataScope_DP.h"
#include "motion.h"
//#include "matrix.h"

#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

//根据自己的电机选择，只能有一个电机类型define生效
//#define GA12N20
#define GM25370
//#define TT130
//#define GB37520

#ifdef GB37520
#define BLC_KP 240
#define BLC_KD 0.75
#define SPD_KP 69
#define SPD_KI 0.345
#define TURN_KP -20
#define TURN_KD -0.6
#endif

#ifdef TT130
#define BLC_KP 360
#define BLC_KD 1.125
#define SPD_KP -72
#define SPD_KI -0.36
#define TURN_KP -20
#define TURN_KD -0.6
#endif

#ifdef GA12N20
#define BLC_KP -240
#define BLC_KD -0.75
#define SPD_KP -72
#define SPD_KI -0.36
#define TURN_KP -20
#define TURN_KD -0.6
#endif

#ifdef GM25370
#define MECHI -5
#define BLC_KP 400
#define BLC_KD 2.2
#define SPD_KP 60
#define SPD_KI 0.3
#define SPD_KD 1
#define TURN_KP 1000
#define TURN_KD 0
#define TURN_KI 10
#define POSI_KP 0
#endif

/*

#ifdef GM25370
#define BLC_KP 240
#define BLC_KD 0.75
#define SPD_KP -60
#define SPD_KI -0.3
#define TURN_KP -20
#define TURN_KD -0.6
#endif

*/

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

/////////////////////////////////////////////////////////////////
//Ex_NVIC_Config专用定义
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 

#define FTIR   1  //下降沿触发
#define RTIR   2  //上升沿触发

//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

//角度是弧度制，坐标单位为厘米。
typedef struct __state_t{
    float x, y, theta;//position and the direction of the car
    float pitch,roll,yaw;//???(???)
		float pitch_rad; 
    float aacx,aacy,aacz;//??????????
    short gyrox,gyroy,gyroz;//???????
    float v_left;//velocity of the left wheel
    float v_right;// m/s
    float R;//???? Radius of motion trajectory
    float v;//???? velocity of the car
    float w;// angular velocity of the car
    u8 move_cmd;
}state_t;

typedef struct __output_t{
    
    int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_Kp;
    int oled_v_pwm, oled_v_I, oled_up_pwm, oled_turn_pwm;
}output_t;

enum direction {back, back_right, right, front_right, front, front_left, left, back_left};

extern float Voltage;  												//电池电压采样相关的变量
extern float pitch,roll,yaw; 										//欧拉角
extern float aacx,aacy,aacz;										//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;									    //陀螺仪原始数据
extern u8 Mode;      			//模式
extern int Uart_Receive;
extern int   Encoder_Left,Encoder_Right;                            //左右编码器的脉冲计数
extern int 	 Moto1,Moto2;										    //计算出来的最终赋给电机的PWM
extern int Velocity,Turn;
extern u8 Fore,Back,Left,Right;
extern u8 TkSensor;
extern float SR04_Distance;
void NVIC_Configuration(void);//中断优先级设置
void Tracking_Init(void);

extern int  oled1;
extern float oled_A,oled_B,oled_C,oled_D,oled_E,oled_F;
extern state_t past,next,now;
#endif

