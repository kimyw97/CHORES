/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "freertos.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx_data;                   // 1바이트 수신용
char rx_cmd_buffer[CMD_BUFFER_SIZE];  // 전체 문자열 버퍼
uint8_t rx_index = 0;             // 버퍼 인덱스

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void setMotorMode(DriveMode mode);
void setMotorSpeed(char motor_position, int speed);
void motorStart();
void motorShutdown();
void startEncoder(TIM_HandleTypeDef *htim);
int32_t readEncoder(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int current_left_pwm;
int current_right_pwm;
int16_t left_encoder_prev_count = 0;
int32_t left_encoder_total_count = 0;
int16_t right_encoder_prev_count = 0;
int32_t right_encoder_total_count = 0;
extern QueueHandle_t MotorSpeedQueue;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart4, &rx_data, 1);
	BSP_GYRO_Init();

	// 가속도 초기화
	BSP_ACCELERO_Init();
	motorStart();
	setMotorMode(FORWARD);
	motorShutdown();
	startEncoder(&htim3);
	startEncoder(&htim4);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);

	left_encoder_prev_count = __HAL_TIM_GET_COUNTER(&htim3);
	left_encoder_total_count = __HAL_TIM_GET_COUNTER(&htim3);

	right_encoder_prev_count = __HAL_TIM_GET_COUNTER(&htim4);
	right_encoder_total_count = __HAL_TIM_GET_COUNTER(&htim3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */

  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
}

/* USER CODE BEGIN 4 */
void setMotorMode(DriveMode mode) {
	switch (mode) {
	case 0:
		HAL_GPIO_WritePin(GPIOD, L_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, L_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, R_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, R_IN2_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, L_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, L_IN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, R_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, R_IN2_Pin, GPIO_PIN_SET);
		break;
		// 으론쪽으로 회전 => Left Motor 전진 , Right Motor 후진
	case 2:
		HAL_GPIO_WritePin(GPIOD, L_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, L_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, R_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, R_IN2_Pin, GPIO_PIN_SET);
		break;
		// 왼쪽으로 회전 => Left Motor 후진 , Right Motor 전진
	case 3:
		HAL_GPIO_WritePin(GPIOD, L_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, L_IN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, R_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, R_IN2_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOD, L_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, L_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, R_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, R_IN2_Pin, GPIO_PIN_RESET);
		break;
	}
}

void setMotorSpeed(char motor_position, int speed) {
	int pwm = abs(speed) * 10;
	if (motor_position == 'L') {
//		current_left_pwm = speed;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
	} else if (motor_position == 'R') {
//		current_right_pwm = speed;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
	}
}

void motorStart() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void motorShutdown() {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void startEncoder(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

int32_t readEncoder(TIM_HandleTypeDef *htim) {
	int16_t current_count = __HAL_TIM_GET_COUNTER(htim);
	int16_t diff = 0;

	if (htim->Instance == TIM4) {  // Right encoder
		diff = current_count -right_encoder_prev_count;

		// 오버플로우 / 언더플로우 보정
		if (diff > 30000)
			diff -= 65536;
		else if (diff < -30000)
			diff += 65536;

		right_encoder_total_count += diff;
		right_encoder_prev_count = current_count;
		return right_encoder_total_count;
	} else if (htim->Instance == TIM3) {  // Left encoder
		diff = current_count - left_encoder_prev_count;

		// 오버플로우 / 언더플로우 보정
		if (diff > 30000)
			diff -= 65536;
		else if (diff < -30000)
			diff += 65536;

		left_encoder_total_count += diff;
		left_encoder_prev_count = current_count;
		return left_encoder_total_count;
	} else {
		return 0;  // 지원하지 않는 타이머
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		// 큐에 수신 바이트 삽입
		// 전달 받은 데이터에 따라 넣는 큐가 다름
		if (rx_data == '\n') {
			rx_cmd_buffer[rx_index] = '\0';  // 문자열 종료
			CommandMessage msg;
			strlcpy(msg.cmd, rx_cmd_buffer, CMD_BUFFER_SIZE);

			// 명령 종류 판별 및 큐 전송
			if (strncmp(msg.cmd, "L", 1) == 0) {
				xQueueSendFromISR(MotorSpeedQueue, &msg,
						&xHigherPriorityTaskWoken);
//			} else if (strncmp(msg.cmd, "S", 1) == 0) {
//				xQueueSendFromISR(ServoQueue, &msg, &xHigherPriorityTaskWoken);
//			} else if (strncmp(msg.cmd, "U", 1) == 0
//					|| strncmp(msg.cmd, "D", 1) == 0) {
//				xQueueSendFromISR(StepperQueue, &msg,
//						&xHigherPriorityTaskWoken);
			}
			rx_index = 0;  // 버퍼 초기화
		} else {
			if (rx_index < CMD_BUFFER_SIZE - 1) {
				rx_cmd_buffer[rx_index++] = rx_data;
			} else {
				rx_index = 0;  // overflow 방지
			}
		}

		// 다시 수신 시작
		HAL_UART_Receive_IT(&huart4, &rx_data, 1);

		// 필요 시 context switch
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
