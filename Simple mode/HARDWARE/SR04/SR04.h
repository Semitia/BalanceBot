#ifndef __UltrasonicWave_H
#define __UltrasonicWave_H
/**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
 微信公众号【大鱼机器人】
 后台回复【平衡小车】：获取平衡小车全套DIY资料
 后台回复【电子开发工具】：获取电子工程师必备开发工具
 后台回复【电子设计资料】：获取电子设计资料包
 知乎：张巧龙 
**************************************************************************/

#include "sys.h" 

void SR04_Configuration(void);               //对超声波模块初始化
void SR04_StartMeasure(void);                //开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
void Timer4_Init(u16 arr,u16 psc);
#endif /* __UltrasonicWave_H */

