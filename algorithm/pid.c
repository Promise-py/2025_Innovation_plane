
#include "pid.h"
#include "Movebase.h"

// 对变量进行范围限制
float PID_abs_limit(float a, float ABS_MAX)
{
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}

void PID_parameter_init(pid *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone)  
{  
	pp->Integralmax = Integralmax;
	pp->outputmax = outputmax;
    pp->Proportion = Kp;
	pp->Integral   = Ki;
	pp->Derivative = Kd;
    pp->DError = pp->Error = pp->SumError = pp->output = pp->LastError = pp->PrevError = pp->errormax = 0.0f;
	pp->first_flag = 1;
	pp->deadzone = deadzone;
} 

// 位置式PID,直接传入误差
void PID_position_PID_calculation_by_error(pid *pp, float error)  
{   
	if(pp->first_flag == 1)
	{
		pp->LastError = error;
		pp->PrevError = error;
		pp->first_flag = 0;
	}	
	
	pp->Error =  error;          
	pp->SumError += pp->Error;                      
	pp->DError = pp->Error - pp->LastError;
	
	pp->output =  pp->Proportion * pp->Error +   \
								PID_abs_limit(pp->Integral * pp->SumError, pp->Integralmax ) +   \
								pp->Derivative * pp->DError ;  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax; 
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}

	// last_out=pp->output;
}


