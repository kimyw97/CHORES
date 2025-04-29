#ifndef __DELAY_US_H__
#define __DELAY_US_H__

#include "stm32f4xx_hal.h"

uint32_t DWT_Delay_Init(void);
void delay_us(uint32_t us);

#endif
