/*******************************************************************
 *@title PID ���ƺ���
 *@brief �����ĺ����е�����������PID��ʼ����PID�����������?
 *@brief ��ʷ�޸����ݣ�
 *
 *       δ������ݣ�?
 *       
 *@time  2016.10.13
 *@editorС��&zin
 *�ɿذ���QQȺ551883670,����759421287@qq.com
 ******************************************************************/
#include "pid.h"
PidObject pidRateX;
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch;
PidObject pidRoll;
PidObject pidYaw;



void pidRest(PidObject *pid)
{
	  	pid->integ = 0;
	    pid->prevError = 0;
	    pid->out = 0;
		pid->offset = 0;
}

/**************************************************************
 * @brief PID算法更新
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 ***************************************************************/	
void pidUpdate(PidObject* pid,const float dt)
{
	 float error;
	 float deriv;
	
    error = pid->desired - pid->measured; 
    pid->integ += error * dt;	
	
	pid->integ = LIMIT(pid->integ,0.001,2.9); //限幅
    deriv = (error - pid->prevError)/dt; 
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;
	
	//pid->out = LIMIT(pid->out,0.001,2.9);	//	
    pid->prevError = error;  	
}

/**************************************************************
 *  CascadePID
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //����PID
{	 
	pidUpdate(pidAngE,dt);    //�ȼ����⻷
	pidRate->desired = pidAngE->out;
	pidUpdate(pidRate,dt);    //�ټ����ڻ�	
}

/*******************************END*********************************/


