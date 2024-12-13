#ifndef __MOVEBASE_H
#define __MOVEBASE_H
#include <stdlib.h>
#include <math.h>
#include "stdint.h"
#include "main.h"
float ABS(float a);
unsigned char calculate_xor_checksum(const unsigned char* data, uint8_t length);
uint8_t PointTrace(float POS_X,float POS_Y,float POS_Z);
uint8_t StartFly(void);
uint8_t Fly_KEEP(float height);
void Fly_init(void);
uint8_t LockZ(void);
#endif