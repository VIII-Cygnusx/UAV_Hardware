#ifndef ANTOTC_H_
#define ANTOTC_H_
#include "Param&init.h"
/* 
上位机发送部分
 */
extern enum ANTO_SEND{			//匿名上位机协议
	ANTO_STATUS     =0x03,	
	ANTO_IMU	    =0X01,
	ANTO_TARGET_POS =0X0A,
	ANTO_TARGET_SPE =0X0B,
	ANTO_MOTOR_PWM  =0X20,
	ANTO_RC_NUM		=0X21,
	ANTO_QUA		=0X04,
};
#define BYTE0(temp)	   (*(char*)(&temp))
#define BYTE1(temp)	   (*((char*)(&temp)+1))

void ANTO_Send(const enum ANTO_SEND FUNCTION);	//发送协议
void example_anto_send(void);
#endif
