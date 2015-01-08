// Check for compatible MCU
#ifndef __AVR_ATmega328P__
//#error 'INVALID MCU DEFINED'
#endif

#ifndef _MPU9150_H_
#define _MPU9150_H_

#include <stdint.h>

int MPU9150_init();
int MPU9150_Read();
void update_DCM(double t);
void get_DCM(double ret[][3]);


#endif
