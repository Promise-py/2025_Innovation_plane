
#include "measure.h"
 
float distance;      //测量距离
float last_distance;  //上一次测量距离
uint8_t count_10;   //计数器
// uint8_t second;   //秒计数器

uint32_t Buf[3] = {0};   //存放定时器计数值的数组
uint8_t  Cnt = 0;    //状态标志位
uint32_t high_time;   //超声波模块返回的高电平时间 ms

void delay_us(uint32_t us)
{
    __HAL_TIM_SetCounter(&htim1, 0);
    __HAL_TIM_ENABLE(&htim1);
    while(__HAL_TIM_GetCounter(&htim1) < us);
		/* Disable the Peripheral */
    __HAL_TIM_DISABLE(&htim1);
}

//读取距离
void HCSR04_GetData(void)
{
    TRIG_H;    //发送信号
    delay_us(20);
    TRIG_L;

    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);  //等待高电平

    __HAL_TIM_SetCounter(&htim3, 0);
    count_10=0; 
    __HAL_TIM_ENABLE(&htim3);

    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);  //等待低电平

    __HAL_TIM_DISABLE(&htim3);
    high_time = count_10*10 + __HAL_TIM_GetCounter(&htim3)/100;  //计算高电平时间
    distance = high_time * 17;  //计算距离
    xQueueSendToFrontFromISR(HIGH_RxPort, &distance, 0);
    // else xQueueSendToFrontFromISR(HIGH_RxPort, &distance, 0);
    last_distance = distance;
         	
}
 
//中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)// 判断触发的中断的定时器为TIM3
	{
		count_10++;
	}

  if(htim==&htim2)
  {
    
  }
}