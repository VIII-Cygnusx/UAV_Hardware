#ifndef ESC_H_
#define ESC_H_
#include "Param&init.h"
#define u8 uint8_t
#define u16 uint16_t


#define count_MAX 1000
#define MOTOR_PWM_TEXT  220

/*锟斤拷锟斤拷锟斤拷锟街�*/
#define MOTOR1_MIN  1800
#define MOTOR2_MIN  1800
#define MOTOR3_MIN  1800
#define MOTOR4_MIN  1800

/*锟斤拷锟酵ｏ拷锟街�*/
#define MOTOR1_PWM_STOP  1900
#define MOTOR2_PWM_STOP  1900  
#define MOTOR3_PWM_STOP  1900  
#define MOTOR4_PWM_STOP  1900  


#define MOTOR1_PWM_MIN  2100  //220
#define MOTOR2_PWM_MIN  2100  //220
#define MOTOR3_PWM_MIN  2100  //220
#define MOTOR4_PWM_MIN  2100  //220  

#define MOTOR1_PWM_MAX  3600
#define MOTOR2_PWM_MAX  3600
#define MOTOR3_PWM_MAX  3600
#define MOTOR4_PWM_MAX  3600
void Motor_judgment(u16 val,int temp,int mode);
void Unlock_pwm(uint8_t temp);
void Set_MOTOR_MAX(uint16_t temp);
void Motor_Unlock(void);

void sport_motor(int16_t pwm1,int16_t pwm2,int16_t pwm3,int16_t pwm4);

void MOTOR_TEXT();



#endif

