#include "canfestival_timer.h"

void setTimer(TIMEVAL value)
{
	htim4.Instance->ARR = htim4.Instance->CNT + value;
}


TIMEVAL getElapsedTime(void)
{
	return htim4.Instance->ARR;
}
