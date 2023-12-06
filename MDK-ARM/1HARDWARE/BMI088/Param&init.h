#ifndef PARAM_INIT_H_
#define PARAM_INIT_H_
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "stdbool.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "dma.h"
#include "BMI088.h"
#include "BMI08X.h"
#include "BMI08X_define.h"
#include "imu.h"
#include "myMath.h"
#include "filter.h"
#include "kalman.h"
#include "nrf24l01.h"
#include "pid.h" 
#include "control.h"
#include "ESC.h"
#include "ANTOTC.h"



/* 
硬件移植部分
 */
#define ALL_LINE          htim3	            //总程序中断总中断设置周期务必与陀螺仪带宽一致，当前IMU带宽:200hz，当前总周期:200hz
#define ESC_PWM_LINE      htim2		        //由tim提供pwm向ESC
#define NRF24L01_SPI_LINE hspi1   			//24L01SPI总线<---初始化必设置
#define BMI088_SPI_LINE   hspi2			    //bmi088陀螺仪spi总线<---初始化必设置
#define SEND_UART         huart1            //上位机发送线
#define BMI088_GPIO_PORT  CS1_GPIO_Port	    //bmi088设备使能总线 <---初始化必设置



/* 
姿态算解硬件部分
 */
extern struct bmi08x_dev dev;			//bmi088设备设置     <---初始化必设置
extern struct bmi08x_sensor_data BMI088_ACC_O; //bmi088加速度原始数据
extern struct bmi08x_sensor_data BMI088_GYRO_O; //bmi088陀螺仪原始数据
extern struct bmi08x_sensor_data_f BMI088_ACC_F;	 //算解出来的浮点数据
extern struct bmi08x_sensor_data_f BMI088_ACC_F_FIX; //算解出来的浮点数据修正
extern struct bmi08x_sensor_data_f BMI088_GYRO_F; 	 //算解出来的浮点数据
extern struct bmi08x_sensor_data_f BMI088_GYRO_F_FIX;//算解出来的浮点数据修正
extern struct Quaternion NumQ;			//四元数
extern struct _1_ekf_filter ekf[6];		//存放卡尔曼滤波参数
extern float KpDef;					//Mahony姿态算解 P
extern float KiDef;					//Mahony姿态算解 I
extern struct _st_AngE Angle;			//存放欧拉角,供全局使用
#define ACC_USE_3G (3.0f*9.80665f)/32768.0f			//此为IMU使用量程设置
#define ACC_USE_6G (6.0f*9.80665f)/32768.0f
#define ACC_USE_12G (12.0f*9.80665f)/32768.0f
#define ACC_USE_24G (24.0f*9.80665f)/32768.0f
#define GYRO_USE_125_DPS (float)(125.0f/32768.0f)
#define GYRO_USE_250_DPS (float)(250.0f/32768.0f)
#define GYRO_USE_500_DPS (float)(500.0f/32768.0f)
#define GYRO_USE_1000_DPS (float)(1000.0f/32768.0f)
#define GYRO_USE_2000_DPS (float)(2000.0f/32768.0f)

/* 
遥测部分
 */
extern struct _st_Remote Remote; 		//存放遥测数据
extern int16_t MAX_THR;                 //最大油门
extern int16_t MIN_THR;                 //最小油门
extern int16_t motor_PWM_Value[4];      //存放油门值

void pid_param_Init(void);              //pid参数初始化                  
void BMI088_param_set(void);            //BMI088参数设置
void ALL_Init(void);					//初始化所有
#endif 

