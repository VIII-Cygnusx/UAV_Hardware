#ifndef __CONTROL_H
#define __CONTROL_H

#include "Param&init.h"


 

#define MOTOR1 motor_PWM_Value[0] 
#define MOTOR2 motor_PWM_Value[1] 
#define MOTOR3 motor_PWM_Value[2] 
#define MOTOR4 motor_PWM_Value[3] 


extern void FlightPidControl(float FightControlTime);
extern void MotorControl(void);
#endif


