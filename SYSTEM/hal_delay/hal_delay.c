#include "hal_delay.h"
#include "usart.h"	

// 毫秒延时
void HAL_Delay_us(uint32_t nus)
{
	//设置定时1us中断�??�??
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
    //调用系统自带的延时函�??
	HAL_Delay(nus - 1);
    //将定时中断恢复为1ms中断
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
}
