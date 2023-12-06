/* 
注意注意，本程序需要先开遥控器再打开飞控，除非你想死，目前待优化中！！！！！！！！！

硬件接线：
IMU:

NRF24L01:

ESC:


配置bmi088时需要设置
spi-> 速度<=10mbps CPOL:High CPHA:2 
使能端口-> 设置名字L:CS1(加速度芯片)  CS2(陀螺仪芯片)
修改宏定义GPIO
配置nrf24l01注意
 */

#include "Param&init.h"


#ifdef __GNUC__
     #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
 #else
     #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
 #endif /* __GNUC__*/
 PUTCHAR_PROTOTYPE
 {
     HAL_UART_Transmit(&SEND_UART, (uint8_t *)&ch,1,0xFFFF);
     return ch;
 }
#ifdef __GNUC__
  #define GETCHAR_PROTOTYPE int __io_getchar()
#else
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
GETCHAR_PROTOTYPE{
  uint8_t ch;
  HAL_UART_Receive(&SEND_UART,(uint8_t*)&ch,1,HAL_MAX_DELAY);
  return ch;
}


struct bmi08x_dev dev = {		
        .accel_id = CS1_Pin,
        .gyro_id = CS2_Pin,
        .intf = BMI08X_SPI_INTF,  
        .read = &stm32_spi_read,  //user_spi_read,  
        .write = &stm32_spi_write,//user_spi_write,  
        .delay_ms = &HAL_Delay	  //user_delay_milli_sec
};
struct bmi08x_sensor_data BMI088_ACC_O;   //int_16		//原始值
struct bmi08x_sensor_data_f BMI088_ACC_F; //float		//算解值m/s^2
struct bmi08x_sensor_data_f BMI088_ACC_F_FIX={.x=0,		//存储零漂值
											  .y=0,
											  .z=0};
struct bmi08x_sensor_data BMI088_GYRO_O;  //int_16		//原始值											  
struct bmi08x_sensor_data_f BMI088_GYRO_F;//float		//算解值弧度制
struct bmi08x_sensor_data_f BMI088_GYRO_F_FIX={.x=0,	//存储零漂值
                                               .y=0,
                                               .z=0};
struct Quaternion NumQ = {1, 0, 0, 0};					//存放四元数
struct _st_AngE Angle={0, 0, 0}; 	//float				//存放当前姿态角
struct _st_Remote Remote = {0, 0, 0, 0, 0, 0, 0, 0}; 	//存放算解出的遥控器数据





/* 一维卡尔曼协方差参数 */
struct _1_ekf_filter ekf[6] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};		
/* Mahony IMU pi参数 */
float KpDef = 270.0f ; //200.0f
float KiDef = 0.002f;  //0.002d

int16_t MAX_THR=2000;
int16_t MIN_THR=1000;
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 * @warning 当前pid的p值太小大致在0.3-0.6之间调整p值
 **********************************************************************/
void pid_param_Init(void){
	pidRateX.kp = 0.35f;
	pidRateY.kp = 0.35f;
	pidRateZ.kp = 1.0f;
	
	pidRateX.ki = 0.05f;
	pidRateY.ki = 0.05f;
	pidRateZ.ki = 0.7f;

	pidRateX.kd = 0.02f;
	pidRateY.kd = 0.02f;
	pidRateZ.kd = 0.1f;	
	
	pidPitch.kp = 10.0f;
	pidRoll.kp = 10.0f;
	pidYaw.kp = 8.0f;	

}

/* BMI088芯片设置 */
void BMI088_param_set(void){
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, CS1_Pin|CS2_Pin, GPIO_PIN_SET);
    bmi088_init(&dev);
    bmi08a_soft_reset(&dev);
	//bmi08a_get_power_mode(&dev);
	//bmi08a_get_meas_conf(&dev);
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;		
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;	//访问采样率200hz
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;	//若更改量程请前往 bmi08a_get_data 函数修改量程
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE; 	//启动powermanager
	bmi08g_soft_reset(&dev);
	bmi08a_set_power_mode(&dev);	  
    bmi08a_set_meas_conf(&dev);
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	bmi08g_set_power_mode(&dev);
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_64_ODR_200_HZ;//访问采样率200hz
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_500_DPS;	//若更改量程请前往 bmi08g_get_data 函数修改量程
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_64_ODR_200_HZ;	
	bmi08g_set_meas_conf(&dev);
}
int16_t motor_PWM_Value[4];//

void ALL_Init(void){
	BMI088_param_set();			//BMI088IMU设置
 	NRF24L01_init();			//NRF24L01芯片设置
	ESC_init();					//电调设置
 	Motor_Unlock();				//电机设置
  	pid_param_Init();			//PID参数初始化
	imu_land_set();				//零漂处理
	HAL_TIM_Base_Start_IT(&ALL_LINE);					//开启定时器中断
	__HAL_TIM_CLEAR_IT(&ALL_LINE, TIM_IT_UPDATE);		//清除所有中断信号
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //固定200hz的带宽算解姿态 优先级:1
{
  /* Prevent unused argument(s) compilation warning */
	RC_Analy();  									//获取遥感
	bmi08a_get_data(&BMI088_ACC_O,&dev);			//获取加速度值
	bmi08g_get_data(&BMI088_GYRO_O,&dev);			//获取角速度值
	filter();										//滤波
	GetAngle(0.005f);    							//获得角度0.005f为时间对应200hz
	FlightPidControl(0.005f);						//pid控制0.005f为时间对应200hz
	MotorControl();									//控制电机转数
  	UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

