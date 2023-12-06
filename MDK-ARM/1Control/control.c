/*******************************************************************
* @brief 此包为飞控控制逻辑包，此飞行模式为跟随遥感输出姿态角模式，后面会更新其他飞行模式
* @warning 目前为测试版，需谨慎使用
* @date 2023/10/25
* @version 1.0.1-test
******************************************************************/
#include "control.h"

PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //???????�??????????????pid???��?????????????????????PID????????  ??????????????��pid???????????????????��????????????
};
/****************************************************************************
 *  @author CYGNUSX
 *  @brief    
 *  @date  2023 8/27
 *  
 * 
 ****************************************************************************/
void  FlightPidControl(float dt){


			const float roll_pitch_ratio = 0.02f;
			const float yaw_ratio =   0.3f;		

			pidRoll.measured = Angle.roll;
			pidPitch.measured = Angle.pitch;
			pidYaw.measured = Angle.yaw;	
			pidRoll.desired = -(Remote.roll-1500.0f)*roll_pitch_ratio+0.06f; //将遥杆值作为飞行角度的期望值
			if(fabs(pidRoll.desired)<0.08f) pidRoll.desired =0.0f;			 //消除遥感电位器的归零误差
			pidPitch.desired =-(Remote.pitch-1500.0f)*roll_pitch_ratio+0.54f;	
			if(fabs(pidPitch.desired)<0.08f)pidPitch.desired=0.0f;			
			pidYaw.desired =-(Remote.yaw-1500.0f)*roll_pitch_ratio+0.54f;	
			if(fabs(pidYaw.desired)<0.5f)pidYaw.desired=0.0f;	
			/* if(Remote.yaw>1800)			pidYaw.desired = yaw_ratio;			 //yaw轴油门严格阈值控制
			else if(Remote.yaw <1180)	pidYaw.desired += yaw_ratio;		 */
      		pidRateX.measured = BMI088_GYRO_F.x; 
			pidRateY.measured = BMI088_GYRO_F.y;
			pidRateZ.measured = BMI088_GYRO_F.z;

		 	pidUpdate(&pidRoll,dt);    	//外环角度环
			pidUpdate(&pidPitch,dt);  
			pidUpdate(&pidYaw,dt);	

			//printf("dePITCH=%4f   deROLL=%4f",pidPitch.desired,pidRoll.desired);		//遥控器范围0-20°
			//printf("pitchde=%4f  rollde=%4f  yawde=%4f  thr=%4d\n",pidPitch.measured,pidRoll.measured,pidYaw.measured,(int16_t)Remote.thr);
			//printf("pitchde=%4f  rollde=%4f  yawde=%4f  thr=%4d\n",pidPitch.desired,pidRoll.desired,pidYaw.desired,(int16_t)Remote.thr);				


		
			pidRateX.desired  = pidRoll.out;
			pidRateY.desired  = pidPitch.out;  
			pidRateZ.desired  = pidYaw.out;
			pidUpdate(&pidRateX,dt);    //内环角速度环
			pidUpdate(&pidRateY,dt);
			//pidUpdate(&pidRateZ,dt);		//不考虑yaw轴
			
			//printf("rout=%4f    yout=%4f    pout=%4f\n",pidRoll.out,pidYaw.out,pidPitch.out);
			 
			//printf("xout=%4f   yout=%4f   zout=%4f   \n",pidRateX.out,pidRateY.out,pidRateZ.out);

}


void MotorControl(void){
	if(Remote.AUX1==2000){    //紧急刹车
      __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,1000);   
      __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,1000);
      __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,1000);
      __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,1000);
	}
	if(Remote.AUX2==2000){		//重置校准imu,pid
		imu_land_set();
		imu_rest();
		pidRest(&pidPitch);
		pidRest(&pidRoll);
		pidRest(&pidYaw);
		pidRest(&pidRateX);
		pidRest(&pidRateY);	
		pidRest(&pidRateZ);
		Angle.yaw=0.0f;         //这里yaw没有与四元数算解绑定，后期上罗盘会修正
	}
	float thr;
	//油门比例规划
	thr = 850.0f+1.2f*((float)Remote.thr -878);
	MOTOR1 =    + (int16_t)pidRateX.out + (int16_t)pidRateY.out + (int16_t)pidRateZ.out+(int16_t)thr;
	MOTOR2 =    + (int16_t)pidRateX.out - (int16_t)pidRateY.out - (int16_t)pidRateZ.out+(int16_t)thr;
	MOTOR3 =    - (int16_t)pidRateX.out + (int16_t)pidRateY.out - (int16_t)pidRateZ.out+(int16_t)thr;
	MOTOR4 =    - (int16_t)pidRateX.out - (int16_t)pidRateY.out + (int16_t)pidRateZ.out+(int16_t)thr;
	MOTOR1=LIMIT(MOTOR1,900,2000);		//这里留100空挡防止怠速空转
	MOTOR2=LIMIT(MOTOR2,900,2000);
	MOTOR3=LIMIT(MOTOR3,900,2000);
	MOTOR4=LIMIT(MOTOR4,900,2000);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,MOTOR1);   //正式输出给ESC无刷电机
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,MOTOR2);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,MOTOR3);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,MOTOR4);
	//只要当前姿态处于每次遥感输出角度的期望值范围内pid都重置
	if(Angle.pitch<=pidPitch.desired+1.0f || Angle.pitch>=pidPitch.desired-1.0f){pidRest(&pidRateX);pidRest(&pidPitch);}
	if(Angle.roll<=pidRoll.desired+1.0f || Angle.roll>=pidRoll.desired-1.0f){pidRest(&pidRateY);pidRest(&pidRoll);}
	if(Angle.yaw<=pidYaw.desired+1.0f || Angle.yaw>=pidYaw.desired-1.0f){pidRest(&pidRateZ);pidRest(&pidYaw);}
	
	
	
} 

