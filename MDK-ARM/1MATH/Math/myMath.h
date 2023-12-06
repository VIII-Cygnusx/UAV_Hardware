#ifndef __MY_MATH_H
#define	__MY_MATH_H
#include "Param&init.h"

extern const float M_PI;
extern const float AtR;
extern const float RtA;
extern const float Gyro_G;
extern const float Gyro_Gr; 

#define PIhalf 1.57079632679489661923f          //定义pi/2
#define PI 3.1415926535897932384626433832f      //定义pi
#define PI2 6.2831853071795864769252867665F     //定义2*pi
#define pow(a) a*a
#define RAD2DEG(x) ((x)*180.0f/PI)              //弧度转角度
#define DEG2RAD(x) ((x)*PI/180.0f)              //角度转弧度
#define MAX(A,B) ((A)>(B)?(A):(B))             //比较最大
#define MIN(A,B) ((A)<(B)?(A):(B))             //比较最小
#define ABS(x) ((x) > 0 ? (x) : -(x))

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

extern float Q_rsqrt(float number);
double fast_sin(double rad);
double fast_cos(double rad);
float atan2_approx(float y, float x);
#endif /* __Algorithm_math_H */
