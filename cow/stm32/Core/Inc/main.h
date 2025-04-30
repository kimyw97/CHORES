/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define Left_Motor_Encoder_B_Pin GPIO_PIN_1
#define Left_Motor_Encoder_B_GPIO_Port GPIOA
#define Arduino_TX_Pin GPIO_PIN_2
#define Arduino_TX_GPIO_Port GPIOA
#define Arduino_RX_Pin GPIO_PIN_3
#define Arduino_RX_GPIO_Port GPIOA
#define Left_Motor_Encoder_A_Pin GPIO_PIN_5
#define Left_Motor_Encoder_A_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define Trashbin_State_pin_Pin GPIO_PIN_14
#define Trashbin_State_pin_GPIO_Port GPIOB
#define External_Emegency_BTN_Pin GPIO_PIN_15
#define External_Emegency_BTN_GPIO_Port GPIOB
#define Left_Motor_IN1_Pin GPIO_PIN_8
#define Left_Motor_IN1_GPIO_Port GPIOD
#define Left_Motor_IN2_Pin GPIO_PIN_9
#define Left_Motor_IN2_GPIO_Port GPIOD
#define Right_Motor_IN1_Pin GPIO_PIN_10
#define Right_Motor_IN1_GPIO_Port GPIOD
#define Right_Motor_IN2_Pin GPIO_PIN_11
#define Right_Motor_IN2_GPIO_Port GPIOD
#define LD3_Orange_Pin GPIO_PIN_13
#define LD3_Orange_GPIO_Port GPIOD
#define Right_Motor_Encoder_B_Pin GPIO_PIN_6
#define Right_Motor_Encoder_B_GPIO_Port GPIOC
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Servo_TX_Pin GPIO_PIN_10
#define Servo_TX_GPIO_Port GPIOC
#define Servo_RX_Pin GPIO_PIN_11
#define Servo_RX_GPIO_Port GPIOC
#define Raspberrypi_TX_Pin GPIO_PIN_12
#define Raspberrypi_TX_GPIO_Port GPIOC
#define Raspberrypi_RX_Pin GPIO_PIN_2
#define Raspberrypi_RX_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Right_Motor_Encoder_A_Pin GPIO_PIN_5
#define Right_Motor_Encoder_A_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Left_Motor_PWM_Pin GPIO_PIN_7
#define Left_Motor_PWM_GPIO_Port GPIOB
#define Right_Motor_PWM_Pin GPIO_PIN_8
#define Right_Motor_PWM_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
