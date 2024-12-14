#include "Movebase.h"
#include <stdint.h>
#include "command.h"
#include "trilateration.h"
#include "data_pool.h"
#include "pid.h"
#include "usart.h"
#include "measure.h"

UWB_Data data;
INS_t imu_data;
pid point_pid;
pid height_pid;
pid roll_pid;
pid pitch_pid;

float Plane_V[3] = {0,0,0};
uint8_t command[8] = {0x66,0x80,0x80,0x80,0x80,0x00,0x00,0x99};

float ABS(float a)
{
    if(a < 0)
        return -a;
    else
        return a;
}

/*将角度限制在-180°到180°
* @param  angle:要限制的值
* @retval 
*/
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}

void Fly_init(void)
{
    PID_parameter_init(&point_pid,1.0f,0.3f, 0.5f, 127, 0, 50);
    PID_parameter_init(&height_pid,6.0f,0.1f, 0.2f, 127, 0, 25);

    PID_parameter_init(&roll_pid,3.0f,0.2f, 0.2f, 30, 0, 1.5);
    PID_parameter_init(&pitch_pid,3.0f,0.1f, 0.2f, 30, 0, 1.5);
}


int q;


uint8_t Angle_adjust(uint8_t *com,float roll,float pitch)
{
    if(xQueueReceiveFromISR(IMU_RxPort, &imu_data, 0) == pdPASS)
    {
        float roll_error = 0;
        float pitch_error = 0;
	 // 计算误差
        if(imu_data.Roll*roll >= 0)
        {
            roll_error = roll - imu_data.Roll;
        }
   
        else
        {
            if(ABS(imu_data.Roll)+ABS(roll) <= 180) roll_error = roll - imu_data.Roll;
            else 
            {
                AngleLimit(&roll_error);
            }
        }
        
        if(imu_data.Pitch*pitch >= 0)
        {
            pitch_error = pitch - imu_data.Pitch;
        }
        else
        {
            if(ABS(imu_data.Pitch)+ABS(pitch) <= 180) pitch_error = pitch - imu_data.Pitch;
            else 
            {
                AngleLimit(&pitch_error);
            }
        }
        PID_position_PID_calculation_by_error(&roll_pid, roll_error);
        PID_position_PID_calculation_by_error(&pitch_pid, pitch_error);
        *(com+1)=roll_pid.output+135;
        *(com+2)=pitch_pid.output+128;
    }
}

uint8_t LockZ(void)
{
    command[3]= (uint8_t)(160);
    command[6] = calculate_xor_checksum(command+1, 5);
    HAL_UART_Transmit(&huart3, command, 8, 1000);
}

uint8_t Fly_FSM(float height)
{
    float pos_z;
    // HCSR04_GetData();
    // if(xQueueReceiveFromISR(HIGH_RxPort, &pos_z, 0) == pdPASS)
    // {
    //     // q++;
    //     float height_error = 0;
    //     height_error=height-pos_z;
    //     PID_position_PID_calculation_by_error(&height_pid, height_error);

    //     command[4] = 128;//不转动
    //     command[3]= (uint8_t)(height_pid.output+128);
    //     command[6] = calculate_xor_checksum(command+1, 5);
    //     // command[3]= (uint8_t)(180);
    //     // command[3]= (uint8_t)(255);
    // // HAL_UART_Transmit(&huart3, command, 8, 1000);
    // }
    command[4] = 128;//不转动

    if(finish== 1)
    {    
        command[3]= (uint8_t)(190);
        
    }
    else if(finish==2)
    {
        command[3]= (uint8_t)(128);
        Angle_adjust(command, 0, 0);
    }
    else if(finish==3)//
    {
        command[3]= (uint8_t)(128);
        Angle_adjust(command, -5, 0);
    }
    else if(finish==4)//
    {
        command[3]= (uint8_t)(128);
        Angle_adjust(command, 0, 10);
    }
    else if(finish==5)//
    {
        command[3]= (uint8_t)(128);
        Angle_adjust(command, 0, 0);
    }
    command[6] = calculate_xor_checksum(command+1, 5);
    HAL_UART_Transmit(&huart3, command, 8, 1000);
    // command[3]=0xFF;
    // command[6] = calculate_xor_checksum(command+1, 5);
    // HAL_UART_Transmit(&huart3, command, 8, 1000);

    // HAL_Delay(3000);

    // command[3]=0x80;
    // command[6] = calculate_xor_checksum(command+1, 5);
    // HAL_UART_Transmit(&huart3, command, 8, 1000);

    // command[4] = 0x80;//不转动
    // command[3]= (uint8_t)(165);
    // command[6] = calculate_xor_checksum(command+1, 5);
    // // uint8_t up[8] = {0x66, 0x80 ,0x80 ,0xFF ,0x80 ,0x00 ,0x7F ,0x99};
    // HAL_UART_Transmit(&huart3, command, 8, 1000);
    return 1;
}



uint8_t  StartFly(void)
{
    uint8_t Height[8] = {0x66, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x99};
    uint8_t up[8] = {0x66, 0x80 ,0x80 ,0xFF ,0x80 ,0x00 ,0x7F ,0x99};
    uint8_t down[8] = {0x66, 0x80 ,0x80 ,0x00 ,0x80 ,0x00 ,0x80 ,0x99};
    HAL_UART_Transmit(&huart3, Height, 8, 1000);
    HAL_Delay(10);
    HAL_UART_Transmit(&huart3, up, 8, 1000);
    HAL_Delay(10);
    HAL_UART_Transmit(&huart3, Height, 8, 1000);
    HAL_Delay(10);
    HAL_UART_Transmit(&huart3, up, 8, 1000);
    __HAL_TIM_SetCounter(&htim2, 0);
    // HAL_Delay(10);
    // HAL_UART_Transmit(&huart3, down, 8, 1000);
    return 1;
}


float error;

/**
 * @function: PointTrace
 * @brief: ָ立体点追踪
 * @param: x,y,z
 * @param: yaw 锁航向角度
 * @param: priority xyz坐标追踪顺序
 * @retval: 0:ing 1:finfish
 */
uint8_t PointTrace(float POS_X,float POS_Y,float POS_Z)
{
    // YawAdjust(POS_YAW);
    Fly_KEEP(POS_Z);
    if(xQueueReceive(UWB_RxPort, &data, 0) == pdPASS)
    {
        //计算误差
        error = sqrt((data.x - POS_X) * (data.x - POS_X) + (data.y - POS_Y) * (data.y - POS_Y));  // 计算误差
        point_pid.outputmax = 127;
        PID_position_PID_calculation_by_error(&point_pid, error);

        Plane_V[0] =-(float)(point_pid.output * 1.0f*(data.x - POS_X) /error);//x轴
        Plane_V[1] = (float)(point_pid.output * 1.0f*(data.y - POS_Y) /error);//y轴
        // Plane_V[2] = (float)(point_pid.output * 1.0f*(data.z - POS_Z) /error);//z轴

        if(ABS(data.x - POS_X)<50 && ABS(data.y- POS_Y)<50)
        {
            return 1;
        }
        else
        {
            Plane_V[0] = 128+Plane_V[0];
            Plane_V[1] = 128+Plane_V[1];
            // Plane_V[2] = 128+Plane_V[2];
            command[1] = (uint8_t)(Plane_V[0]);
            command[2] = (uint8_t)(Plane_V[1]);

            command[4] = 128;//不转动
            
            return 0;
        }
    }
    command[6] = calculate_xor_checksum(command+1, 5);
    HAL_UART_Transmit(&huart3, command, 8, 1000);

}


unsigned char calculate_xor_checksum(const unsigned char* data, uint8_t length) 
{
    unsigned char checksum = 0;
    for (uint8_t i = 0; i < length; i++) 
    {
        checksum ^= data[i];
    }
    return checksum;
}

