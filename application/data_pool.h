#ifndef __DATA_POOL_H
#define __DATA_POOL_H

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "trilateration.h"
#include "ins_task.h"
#include <stddef.h>
#include "main.h"


void DataPool_Init(void);

extern QueueHandle_t IMU_RxPort;
extern QueueHandle_t UWB_RxPort;
extern QueueHandle_t UART_TxPort;
extern QueueHandle_t HIGH_RxPort;
#endif
