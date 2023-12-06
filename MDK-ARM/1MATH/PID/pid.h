
#ifndef __PID_H
#define __PID_H
#include "Param&init.h"

typedef volatile struct
{
	float desired;     			 //期望值
	float offset;      			 //重置标志位
	float prevError;  		 	 //上一次误差
	float integ;       		 	 //积分
	float kp;           		 //P
	float ki;          			 //I
	float kd;          			 //D
	float IntegLimitHigh;        //积分限幅
	float IntegLimitLow;
	float OutLimitHigh;			 //输出限幅
	float OutLimitLow;	
	float measured;				 //当前值
	float out;					 //输出值

}PidObject;

extern	PidObject pidRateX;
extern	PidObject pidRateY;
extern	PidObject pidRateZ;

extern	PidObject pidPitch;
extern	PidObject pidRoll;
extern	PidObject pidYaw;

extern void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //����PID
extern void pidRest(PidObject *pid); //pid���ݸ�λ
extern void pidUpdate(PidObject* pid,const float dt);  //PID

#endif



