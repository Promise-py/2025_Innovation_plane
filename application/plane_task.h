#pragma once

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "tim.h"
#include "ex_05a_main.h"
#include "data_pool.h"
#include "usart.h"
#include <math.h>
#include "Movebase.h"
#include "measure.h"
#include "trilateration.h"


/*任务节点创建区*/
osThreadId insTaskHandle;
osThreadId uwbTaskHandle;
osThreadId SendTaskHandle;
osThreadId FlyTaskHandle;

/*任务函数声明*/
void StartINSTASK(void const *argument);
void StartUWBTask(void const *argument);
void SendCommandTask(void const *argument);
void FlyTask(void const *argument);
/**
 * @name    oSTASKInit
 * @brief   任务初始化，所有运行的任务都在此初始化
 */
void OSTASKInit(void)
{
    osThreadDef(instask,StartINSTASK,osPriorityAboveNormal,0,1024);
    insTaskHandle = osThreadCreate(osThread(instask),NULL); //设置较高优先级

    // osThreadDef(uwbtask,StartUWBTask,osPriorityAboveNormal,0,1024);
    // uwbTaskHandle = osThreadCreate(osThread(uwbtask),NULL);

    osThreadDef(flytask,FlyTask,osPriorityNormal,0,512);
    FlyTaskHandle = osThreadCreate(osThread(flytask),NULL);
}


/*bmi088数据获取处理 */
void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init();
    for (;;)
    {
        // 1kHz
        INS_Task();
        // HCSR04_GetData();
        osDelay(1);
    }
}


/*UWB数据接收函数*/
void StartUWBTask(void const *argument)
{
    dw_init();
    for (;;)
    {
        // 1kHz
        dw_Receive();
        osDelay(1);
    }
}


// int a=0;
// /*命令发送函数*/
// void SendCommandTask(void const *argument)
// {
//     unsigned char command[8];
//     for (;;)
//     {
//         if(xQueueReceive(UART_TxPort, command, 0) == pdPASS)
//         {
//             a++;
//             HAL_UART_Transmit(&huart3, command, 8, 1000);
//         }
//         osDelay(1);
//     }
// }


/*飞行执行任务*/
void FlyTask(void const *argument)
{
    for (;;)
    {
        // 100Hz
        // PointTrace(1,1,1,1);
        // HAL_UART_Transmit(&huart3, Height, 8, 1000);
        // xQueueSendFromISR(UART_TxPort, Height, 0);
        // HAL_Delay(10);
        FSM_Plane();
        osDelay(10);
    }
}