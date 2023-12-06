#ifndef __IMU_H
#define	__IMU_H
#include "Param&init.h"

struct Quaternion{
  volatile float q0;
  volatile float q1;
  volatile float q2;
  volatile float q3;
};

struct _st_AngE{
	volatile float roll;
	volatile float pitch;
	volatile float yaw;
};
struct V{
	volatile float x;
	volatile float y;
	volatile float z;
	};


void filter(void);
extern void GetAngle(float dt);
extern void imu_rest(void);
void imu_land_set(void);
#endif
