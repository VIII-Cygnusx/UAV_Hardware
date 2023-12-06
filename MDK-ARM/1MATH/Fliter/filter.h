#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

#include "Param&init.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.2831853071795864769252867665F *(t) ) ) ) *( (in) - (out) ))	//低通滤波



#endif /* __Algorithm_filter_H */
