#include "plane_task.h"


void PlaneInit(void)
{
    __disable_irq();

    OSTASKInit();

    __enable_irq();
}

