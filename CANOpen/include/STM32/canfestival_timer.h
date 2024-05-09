#ifndef __CANFESTIVAL_TIMER_H
#define __CANFESTIVAL_TIMER_H
#include "sys.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "canfestival.h"

// void TIMConfig(TIM_TypeDef* TIMx, uint16_t TIM_Period, uint16_t TIM_Prescaler);
void setTimer(TIMEVAL value);
TIMEVAL getElapsedTime(void); 

extern TIM_HandleTypeDef htim4;

#endif
