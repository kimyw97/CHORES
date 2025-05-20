#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "stm32f4xx_hal.h"


void STS3032_WritePosition(uint8_t id, uint16_t position, UART_HandleTypeDef *huart);

#endif
