#include "plane_task.h"


void PlaneInit(void)
{
    __disable_irq();
    
    Fly_init();
    DataPool_Init();
    
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

        case 1:
            if(Fly_KEEP(1000));
        break;
        case 2:
            // LockZ();
        break;
        case 3:

        break;
    }
}

