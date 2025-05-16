/*
 * usart.c
 *
 *  Created on: May 8, 2025
 *      Author: okpjh
 */


#include "usart.h"

UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart3;


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  // Full-Duplex 초기화
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_USART3_UART_Init(void)
 {
   huart3.Instance = USART3;
   huart3.Init.BaudRate = 115200;
   huart3.Init.WordLength = UART_WORDLENGTH_8B;
   huart3.Init.StopBits = UART_STOPBITS_1;
   huart3.Init.Parity = UART_PARITY_NONE;
   huart3.Init.Mode = UART_MODE_TX_RX;
   huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart3.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&huart3) != HAL_OK)
   {
     Error_Handler();
    }
  }
