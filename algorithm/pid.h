#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>
typedef struct _pid
{
	float  Proportion;		   // 比例常数Kp
	float  Integral;		   // 积分常数Ki  
	float  Derivative;		   // 微分常数Kd  
	  float  PrevError; 	   // 上上次的Error[-2]
	float  LastError;		   // 上次的Error[-1]  
	  float  Error; 		   // 当前的Error[0]
	  float  DError;		   // 当前的误差变化率 Error[0]-Error[-1]
	float  SumError;		   //  误差累计值	
	  float  Integralmax;	   //  积分限幅
	  float  output;		   //  pid控制器的输出值
	  float  outputmax; 	   //  pid控制器输出值限幅
	  float  errormax;		   //  误差的最大值限制  
	  uint8_t first_flag;	   //  第一次标志位，用于初始化控制器或特殊处理
	  float  deadzone;		   //  死区值  

}pid;

void PID_position_PID_calculation_by_error(pid *pp, float error);
// 对变量进行范围限制
float PID_abs_limit(float a, float ABS_MAX);
void PID_parameter_init(pid *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone);

#endif 