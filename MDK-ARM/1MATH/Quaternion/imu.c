#include "imu.h"

volatile	 struct V GyroIntegError = {0}; 
	 
void imu_rest(void)
{
	NumQ.q0 =1;
	NumQ.q1 = 0;
	NumQ.q2 = 0;
	NumQ.q3 = 0;	
	GyroIntegError.x = 0;
	GyroIntegError.y = 0;
	GyroIntegError.z = 0;
	Angle.pitch = 0;
	Angle.roll = 0;
}
void filter(void){

	LPF_1_(2.0f,0.005f,BMI088_GYRO_F.x,BMI088_GYRO_F.x);
	LPF_1_(2.0f,0.005f,BMI088_GYRO_F.y,BMI088_GYRO_F.y);
	LPF_1_(2.0f,0.005f,BMI088_GYRO_F.z,BMI088_GYRO_F.z);
	LPF_1_(2.0f,0.005f,BMI088_ACC_F.x,BMI088_ACC_F.x);
	LPF_1_(2.0f,0.005f,BMI088_ACC_F.y,BMI088_ACC_F.y);
	LPF_1_(2.0f,0.005f,BMI088_ACC_F.z,BMI088_ACC_F.z);
  	kalman_1(&ekf[0],(float)BMI088_ACC_F.x);BMI088_ACC_F.x=ekf[0].out;
	kalman_1(&ekf[1],(float)BMI088_ACC_F.y);BMI088_ACC_F.y=ekf[1].out;
	kalman_1(&ekf[2],(float)BMI088_ACC_F.z);BMI088_ACC_F.z=ekf[2].out;
	kalman_1(&ekf[3],(float)BMI088_GYRO_F.x);BMI088_GYRO_F.x=ekf[3].out;
	kalman_1(&ekf[4],(float)BMI088_GYRO_F.y);BMI088_GYRO_F.y=ekf[4].out;
	kalman_1(&ekf[5],(float)BMI088_GYRO_F.z);BMI088_GYRO_F.z=ekf[5].out;	

}
/***********************************************************************
 * @param[in] 
 * @param[out] 
 * @return     
 * @warning 当前yaw值算解出来有问题，需要改进
 **********************************************************************/
void GetAngle(float dt)
{		
	BMI088_ACC_F.x-=BMI088_ACC_F_FIX.x;				
	BMI088_ACC_F.y-=BMI088_ACC_F_FIX.y;
	BMI088_ACC_F.z-=BMI088_ACC_F_FIX.z;
	BMI088_GYRO_F.x-=BMI088_GYRO_F_FIX.x;
	BMI088_GYRO_F.y-=BMI088_GYRO_F_FIX.y;
	BMI088_GYRO_F.z-=BMI088_GYRO_F_FIX.z;		
volatile struct V Gravity,Acc,Gyro,AccGravity;
float NormAcc;	
float NormQuat; 
float HalfTime = dt * 0.5f;


	
 NormAcc = Q_rsqrt(pow(BMI088_ACC_F.x)+ pow(BMI088_ACC_F.y) +pow(BMI088_ACC_F.z));
 Acc.x = BMI088_ACC_F.x * NormAcc;
 Acc.y = BMI088_ACC_F.y * NormAcc;
 Acc.z = BMI088_ACC_F.z * NormAcc;	

 

 	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);			//[2][0]				
 	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);			//[2][1]  
 	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);		//[2][2]	

 	AccGravity.x += (Acc.y * Gravity.z - Acc.z * Gravity.y);			//ex p
 	AccGravity.y += (Acc.z * Gravity.x - Acc.x * Gravity.z);
 	AccGravity.z += (Acc.x * Gravity.y - Acc.y * Gravity.x);





 	GyroIntegError.x += AccGravity.x * KiDef* dt;                     //in i
 	GyroIntegError.y += AccGravity.y * KiDef* dt;
 	GyroIntegError.z += AccGravity.z * KiDef* dt;

    Gyro.x += BMI088_GYRO_F.x * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//������
    Gyro.y += BMI088_GYRO_F.y * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z += BMI088_GYRO_F.z * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		

	
	NumQ.q0 +=(-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	NumQ.q1 +=( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	NumQ.q2 +=( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	NumQ.q3 +=(-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;

NormQuat = Q_rsqrt(pow(NumQ.q0) + pow(NumQ.q1) + pow(NumQ.q2) + pow(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
	


	{
		 
			#ifdef	YAW_GYRO
				Angle.yaw = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				float yaw_G = BMI088_GYRO_F.z * Gyro_G;
				if((yaw_G > 0.5f) || (yaw_G < -0.5f)) 
				{
					Angle.yaw  += yaw_G * dt*4.0f;			
				}
			#endif				
			Angle.pitch = RAD2DEG(fast_sin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3));
			Angle.roll   = RAD2DEG(atan2_approx(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2));  
			
	}
}
/* 
imu零漂处理
 */
void imu_land_set(void){
	volatile float ax,ay,gx,gy;
	for(int i=0;i<200;i++){
	bmi08a_get_data(&BMI088_ACC_O,&dev);
	bmi08g_get_data(&BMI088_GYRO_O,&dev);
	LPF_1_(2.0f,0.005f,BMI088_GYRO_F.x,BMI088_GYRO_F.x);
	LPF_1_(2.0f,0.005f,BMI088_GYRO_F.y,BMI088_GYRO_F.y);
	LPF_1_(2.0f,0.005f,BMI088_ACC_F.x,BMI088_ACC_F.x);
	LPF_1_(2.0f,0.005f,BMI088_ACC_F.y,BMI088_ACC_F.y);
	ax+=BMI088_ACC_F.x;
	ay+=BMI088_ACC_F.y;
	gx+=BMI088_GYRO_F.x;
	gy+=BMI088_GYRO_F.y;
	}
	BMI088_ACC_F_FIX.x=ax/200.0f;
	BMI088_ACC_F_FIX.y=ay/200.0f;
	BMI088_GYRO_F_FIX.x=gx/200.0f;
	BMI088_GYRO_F_FIX.y=gy/200.0f;
}




