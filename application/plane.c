#include "plane_task.h"
#include "main.h"
#include "trilateration.h"
#include "tim.h"



void PlaneInit(void)
{
    __disable_irq();
    
    Fly_init();
    DataPool_Init();

    KalmanFilter_Init(&uwbkx);
    KalmanFilter_Init(&uwbky);
    HAL_TIM_Base_Start_IT(&htim2);
    OSTASKInit();


    __enable_irq();
}
 
uint8_t finish=0;//致敬九期天路机器人
void FSM_Plane(void)
{
    switch(finish)
    {
        case 0:
            if(StartFly())finish++;
        break;

        default:
            Fly_FSM(1);
        break;
    }
}


