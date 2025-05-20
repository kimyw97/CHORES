/*
 * usart.h
 *
 *  Created on: May 8, 2025
 *      Author: okpjh
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_USART_H_ */
