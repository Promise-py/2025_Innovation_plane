
#include "data_pool.h"
#include "command.h"
QueueHandle_t IMU_RxPort;
QueueHandle_t UWB_RxPort;
QueueHandle_t UART_TxPort;
QueueHandle_t HIGH_RxPort;


uint8_t Height[8];
float h;
void DataPool_Init(void)
{
    IMU_RxPort = xQueueCreate(1, sizeof(INS_t));
    UWB_RxPort = xQueueCreate(1, sizeof(UWB_Data));
    UART_TxPort = xQueueCreate(1, sizeof(Height));
    HIGH_RxPort = xQueueCreate(1, sizeof(float));
}


