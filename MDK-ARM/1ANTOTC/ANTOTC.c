#include "ANTOTC.h"
/***********************************************************************
 * @brief 适配匿名上位机v7，目前只有发送功能，待完善
 * @param[in] 
 * @param[out] 
 * @return [viod]
 * @date 2023/10/25
 * @version 1.0.0
 **********************************************************************/
void ANTO_Send(const enum ANTO_SEND FUNCTION) 
{
	uint8_t Anto[20]=0;

	switch(FUNCTION)
	{
		case ANTO_QUA:     //发送四元数
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_QUA;
			Anto[3] = 9;
			int16_t W0=(int16_t)(NumQ.q0*10000.0f);
			Anto[4] = BYTE0(W0);
			Anto[5] = BYTE1(W0);
			int16_t Q1;
			Q1=(int16_t)(NumQ.q1*10000.0f);			
			Anto[6] = BYTE0(Q1);
			Anto[7] = BYTE1(Q1);
			int16_t Q2;
			Q2=(int16_t)(NumQ.q2*10000.0f);				
			Anto[8] = BYTE0(Q2);
			Anto[9] = BYTE1(Q2);
			int16_t Q3;
			Q3=(int16_t)(NumQ.q3*10000.0f);				
			Anto[10] = BYTE0(Q3);
			Anto[11] = BYTE1(Q3);
			Anto[12]=0;
			break;	
		case ANTO_RC_NUM:     //遥感油门
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_RC_NUM;
			Anto[3] = 8;
			Anto[4] = BYTE0(Remote.roll);
			Anto[5] = BYTE1(Remote.roll);
			Anto[6] = BYTE0(Remote.pitch);
			Anto[7] = BYTE1(Remote.pitch);
			Anto[8] = BYTE0(Remote.thr);
			Anto[9] = BYTE1(Remote.thr);
			Anto[10] = BYTE0(Remote.yaw);
			Anto[11] = BYTE1(Remote.yaw);
			break;		
		case ANTO_MOTOR_PWM:     //电机pwm
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_MOTOR_PWM;
			Anto[3] = 8;
			Anto[4] = BYTE0(MOTOR1);
			Anto[5] = BYTE1(MOTOR1);
			Anto[6] = BYTE0(MOTOR2);
			Anto[7] = BYTE1(MOTOR2);
			Anto[8] = BYTE0(MOTOR3);
			Anto[9] = BYTE1(MOTOR3);
			Anto[10] = BYTE0(MOTOR4);
			Anto[11] = BYTE1(MOTOR4);
			break;			
		case ANTO_TARGET_SPE:     //pid
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_TARGET_SPE;
			Anto[3] = 6;
			int16_t DEXS=(int16_t)pidRateX.desired;			
			Anto[4] = BYTE0(DEXS);
			Anto[5] = BYTE1(DEXS);
			int16_t DEYS=(int16_t)pidRateY.desired;			
			Anto[6] = BYTE0(DEYS);
			Anto[7] = BYTE1(DEYS);
			int16_t DEZS=(int16_t)pidRateZ.desired;			
			Anto[8] = BYTE0(DEZS);
			Anto[9] = BYTE1(DEZS);
			break;	
		case ANTO_TARGET_POS:     //
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_TARGET_POS;
			Anto[3] = 6;
			int16_t DERP=(int16_t)pidRoll.desired;			
			Anto[4] = BYTE0(DERP);
			Anto[5] = BYTE1(DERP);
			int16_t DEPP=(int16_t)pidPitch.desired;			
			Anto[6] = BYTE0(DEPP);
			Anto[7] = BYTE1(DEPP);
			int16_t DEYP=(int16_t)pidYaw.desired;			
			Anto[8] = BYTE0(DEYP);
			Anto[9] = BYTE1(DEYP);
			break;	
		case ANTO_IMU:     //
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_IMU;
			Anto[3] = 13;
			int16_t AX=(int16_t)BMI088_ACC_F.x;
			Anto[4] = BYTE0(AX);
			Anto[5] = BYTE1(AX);
			int16_t AY=(int16_t)BMI088_ACC_F.y;					
			Anto[6] = BYTE0(AY);
			Anto[7] = BYTE1(AY);
			int16_t AZ=(int16_t)BMI088_ACC_F.z;			
			Anto[8] = BYTE0(AZ);
			Anto[9] = BYTE1(AZ);
			int16_t GX=(int16_t)BMI088_GYRO_F.x;			
			Anto[10] = BYTE0(GX);
			Anto[11] = BYTE1(GX);
			int16_t GY=(int16_t)BMI088_GYRO_F.y;			
			Anto[12] = BYTE0(GY);
			Anto[13] = BYTE1(GY);
			int16_t GZ=(int16_t)BMI088_GYRO_F.z;			
			Anto[14] = BYTE0(GZ);
			Anto[15] = BYTE1(GZ);			
			Anto[16] = 0;
			break;			
		case ANTO_STATUS:     //send angle
			Anto[0] = 0XAA;
			Anto[1] = 0xFF;
			Anto[2] = ANTO_STATUS;
			Anto[3] = 7;
			int16_t now_roll=(int16_t)(-Angle.roll*100.0f);			
			Anto[4] = BYTE0(now_roll);
			Anto[5] = BYTE1(now_roll);
			int16_t now_pitch=(int16_t)(Angle.pitch*100.0f);			
			Anto[6] = BYTE0(now_pitch);
			Anto[7] = BYTE1(now_pitch);
			int16_t now_yaw=(int16_t)(Angle.yaw*100.0f);			
			Anto[8] = BYTE0(now_yaw);
			Anto[9] = BYTE1(now_yaw);
			Anto[10] = 0;
			break;		
	}
	int16_t sumcheck=0;
	int16_t addcheck=0;
	for(int i=0;i<(Anto[3]+4);i++){
		sumcheck+=Anto[i];
		addcheck+=sumcheck;
	}
	Anto[(Anto[3]+4)]=((uint8_t)sumcheck);
	Anto[(Anto[3]+4)+1]=((uint8_t)addcheck);
	for(int i=0;i<(Anto[3]+4)+2;i++)printf("%c",Anto[i]);
}
void example_anto_send(void){
	ANTO_Send(ANTO_STATUS);
    ANTO_Send(ANTO_IMU);
    ANTO_Send(ANTO_TARGET_POS);
    ANTO_Send(ANTO_TARGET_SPE);
    ANTO_Send(ANTO_MOTOR_PWM);
    ANTO_Send(ANTO_RC_NUM);
    ANTO_Send(ANTO_QUA);
}