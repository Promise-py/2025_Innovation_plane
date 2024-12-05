#pragma once

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "tim.h"

/*任务节点创建区*/
osThreadId insTaskHandle;

/*任务函数声明*/
void StartINSTASK(void const *argument);

/**
 * @name    oSTASKInit
 * @brief   任务初始化，所有运行的任务都在此初始化
 */
void OSTASKInit(void)
{
    osThreadDef(instask,StartINSTASK,osPriorityAboveNormal,0,1024);
    insTaskHandle = osThreadCreate(osThread(instask),NULL); //设置较高优先级
}

int a;
void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init();
    for (;;)
    {
        // 1kHz
        INS_Task();
        osDelay(1);
    }
}

