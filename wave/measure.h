#ifndef __MEASURE_H__
#define __MEASURE_H__

#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "data_pool.h"
#include "plane.h"
 
#define TRIG_H  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET)
#define TRIG_L  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET)
 
void delay_us(uint32_t us);
void HCSR04_GetData(void);

extern float distance;      //测量距离
#endif