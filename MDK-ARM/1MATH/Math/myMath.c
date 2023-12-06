/*******************************************************************
 * @brief 数学库，待更新其他功能
 * @date 202/10/25
 * @version 1.0.0
 * 

******************************************************************/
 
#include "myMath.h"
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f*2;	  	//�����ǳ�ʼ������+-2000��ÿ����1 / (65536 / 4000) = 0.03051756*2		
const float Gyro_Gr = 0.0005326f*2;     //������ÿ��,ת������ÿ���� 2*0.03051756	 * 0.0174533f = 0.0005326*2


/**
 * @brief 快速开平方根1/sqrt(x)
 * 适用于对于精度不高的快速运算
 * @note https://www.zhihu.com/question/26287650
 * @param number
 * @return float 
 */
float Q_rsqrt(float number)
{
	long i;
	float x;
	x = number * 0.5F;	
	i = * ( long * ) &number;
	i = 0x5f3759df - ( i >> 1 );
	number = * ( float * ) &i;
	number = number * ( 1.5f - ( x * number * number ) );	//牛顿迭代法
	number = number * ( 1.5f - ( x * number * number ) );
	return number;
}

/**
 * @brief 快速开正弦sin(x),范围0-n
 * @note 
 * @param rad 单位弧度制
 * @return double 
 */
double fast_sin(double rad)
{
	#define sinPolyCoef3 -1.666568107e-1f
	#define sinPolyCoef5  8.312366210e-3f
	#define sinPolyCoef7 -1.849218155e-4f
	#define sinPolyCoef9  2.600054768e-6f
    int32_t xint = rad;
    if (xint < -32 || xint > 32) return 0.0f;                   // Stop here on error input (5 * 360 Deg)
    while (rad >  PI) rad -= (PI2);                             // always wrap input angle to -PI..PI
    while (rad < -PI) rad += (PI2);
    if (rad >  (PIhalf))			rad = (PIhalf) - (rad - (PIhalf));   // We just pick -90..+90 Degree
    else if (rad < -(PIhalf))		rad = -(PIhalf) - ((PIhalf) + rad);
    float x2 = rad * rad;
    return rad + rad * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

/**
 * @brief 快速开余弦cos(x)
 * 范围0-n，思路cos(x)=sin(x+pi/2)
 * @note 
 * @param rad 单位弧度制
 * @return double 
 */
double fast_cos(double rad)
{
	return fast_sin(rad+PIhalf);
}

/**
 * @brief 快速开反正切函数arctan(y,x) 
 * Error max: ca 0,0012 Degree Speedgain around 10us
 * @note 
 * @param y 
 * @param x 
 * @return float 单位弧度制
 */
float atan2_approx(float y, float x)
{
	#define ATANmagik1 -0.05030176426f                                          // Double: -0.05030176425872175099
    #define ATANmagik2 -6.988836621f                                            // Double: -6.9888366207752135
    #define ATANmagik3  3.145599955e-7f                                         // Double:  3.14559995508649281e-7
    #define ATANmagik4  2.844463688f                                            // Double:  2.84446368839622429
    #define ATANmagik5  0.8263997833f                                           // Double:  0.826399783297673451
    #define ATANmagik6  0.1471039134f                                           // Double:  0.1471039133652469065841349249
    #define ATANmagik7  0.6444640677f                                           // Double:  0.644464067689154755092299698
    float res, rsq, absX, absY;
    absX = ABS(x);
    absY = ABS(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    rsq = res * res;
    res = ATANmagik1 * (ATANmagik2 + res) * (ATANmagik3 + res) * (ATANmagik4 + ATANmagik5 * res + rsq) /
          (1.0f + ATANmagik6 * res + ATANmagik7 * rsq);
    if (absY > absX) res = PIhalf - res;
    if (x < 0) res = PI - res;
    if (y < 0) res = -res;
    return res;
}
