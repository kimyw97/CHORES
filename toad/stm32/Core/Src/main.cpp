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
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#ifdef __cplusplus
}
#endif
#include <string.h>
#include "SCS.h"
#include "SCSerial.h"
#include "SCServo.h"
#include <stdint.h>
#include <stddef.h>
#include "SMS_STS.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* USER DEFINE BEGIN PinMapping */
#define Left_Motor_In1_Pin GPIO_PIN_0
#define Left_Motor_In2_Pin GPIO_PIN_1
#define Left_GPIO_Port GPIOB

#define Right_Motor_In1_Pin GPIO_PIN_9
#define Right_Motor_In2_Pin GPIO_PIN_10
#define Right_GPIO_Port GPIOD

#define Front_Infrared1 GPIO_PIN_0
#define Front_Infrared2 GPIO_PIN_1
#define IR_GPIO_Port GPIOC

#define Right_PWM GPIO_PIN14
#define Left_PWM GPIO_PIN13
#define PWM_GPIO_Port GPIOD
/* USER DEFINE END PinMapping */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void send_infrared_feedback();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
SMS_STS servo;

#define RX_BUF_SIZE 128
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;
uint8_t rx_byte;

#define MAX_COMMANDS 6

typedef struct {
    int id;
    int pos;
} ServoCommand;

volatile uint8_t command_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void process_command(char* cmd);
void set_motor_pwm(int left, int right);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
extern "C" int main(void)
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();


  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
   servo.begin(&huart2); // ← 실제 UART 핸들 등록
   for (int i = 1; i <= 6; i++) {
       servo.EnableTorque(i, 1);
   }
   HAL_UART_Receive_IT(&huart3, &rx_byte, 1);  // 인터럽트 기반 수신 시작

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (command_ready == 1) {
	    command_ready = 0;  // 플래그 클리어
	    process_command((char*)rx_buf);
	    memset(rx_buf, 0, sizeof(rx_buf));  // 버퍼 클리어
	}

//	send_infrared_feedback();
//	HAL_Delay(1000);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */



  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init0 1 */


  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_Custom */

  // Right 모터 방향 제어: PD9, PD10
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // Left 모터 방향 제어 또는 기타 출력: PB0, PB1
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 전방 적외선 센서 입력: PC0, PC1
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_Custom */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            rx_buf[rx_index] = '\0';
            command_ready = 1;  //  플래그만 세움
            rx_index = 0;
        } else {
            if (rx_index < RX_BUF_SIZE - 1) {
                rx_buf[rx_index++] = rx_byte;
            } else {
                rx_index = 0;  // 오버플로우 방지
            }
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}


void process_command(char* raw_cmd) {
    const int fixed_speed = 100;
    const int fixed_acc = 50;

   /* AGV 모터 명령인지 확인 후 동작*/
    int left = 0, right = 0;
    if (sscanf(raw_cmd, "L%dR%d", &left, &right) == 2) {
        printf("[AGV] Parsed → L=%d, R=%d\n", left, right);
        set_motor_pwm((int)left,(int)right);  // AGV 모터 동작
        return;  // AGV 명령이면 여기서 끝
    }

    /* 로봇 암 명령 처리 */
    ServoCommand commands[MAX_COMMANDS];
    int command_count = 0;

    for (char* token = strtok(raw_cmd, ",");
         token && command_count < MAX_COMMANDS;
         token = strtok(NULL, ","))
    {
        int id = 0, pos = 0;
        if (sscanf(token, "%d:%d", &id, &pos) == 2) {
            commands[command_count++] = (ServoCommand){id, pos};
            printf("Parsed → ID=%d, POS=%d\r\n", id, pos);
        } else {
            printf("⚠️ Parse error: %s\r\n", token);
        }
    }

    for (int i = 0; i < command_count; i++) {
        servo.RegWritePosEx(commands[i].id, commands[i].pos, fixed_speed, fixed_acc);
        HAL_Delay(50);
    }

    HAL_Delay(100);
    printf("[ACT] Executing Action()\r\n");
    servo.Action();
}


/* --- 모터 PWM + 방향 제어 함수 --- */
void set_motor_pwm(int left, int right)
{
    // 1) PWM 듀티 계산 (0 ~ 1000)
    uint16_t pwm_L = (uint16_t)(fminf(fabsf(left)  * 10.0f, 1000.0f));
    uint16_t pwm_R = (uint16_t)(fminf(fabsf(right) * 10.0f, 1000.0f));

    // 2) 방향 제어 (IN1/IN2)
    // Left
    if (left > 0) {
        // 전진
        HAL_GPIO_WritePin(Left_GPIO_Port, Left_Motor_In1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Left_GPIO_Port, Left_Motor_In2_Pin, GPIO_PIN_RESET);
    } else if (left < 0) {
        // 후진
        HAL_GPIO_WritePin(Left_GPIO_Port, Left_Motor_In1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_GPIO_Port, Left_Motor_In2_Pin, GPIO_PIN_SET);
    } else {
        // 정지→Coast
        HAL_GPIO_WritePin(Left_GPIO_Port, Left_Motor_In1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_GPIO_Port, Left_Motor_In2_Pin, GPIO_PIN_RESET);
    }

    // Right
    if (right > 0) {
        // 전진
        HAL_GPIO_WritePin(Right_GPIO_Port, Right_Motor_In1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Right_GPIO_Port, Right_Motor_In2_Pin, GPIO_PIN_RESET);
    } else if (right < 0) {
        // 후진
        HAL_GPIO_WritePin(Right_GPIO_Port, Right_Motor_In1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Right_GPIO_Port, Right_Motor_In2_Pin, GPIO_PIN_SET);
    } else {
        // 정지→Coast
        HAL_GPIO_WritePin(Right_GPIO_Port, Right_Motor_In1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Right_GPIO_Port, Right_Motor_In2_Pin, GPIO_PIN_RESET);
    }

    // 3) PWM 출력 (ENA/ENB)
    // TIM_CHANNEL_2 → Left_PWM (PD13)
    // TIM_CHANNEL_3 → Right_PWM (PD14)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_L);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_R);
}




/* --- 적외선 센서 상태 전송 함수 --- */
void send_infrared_feedback() {
    uint8_t s1 = HAL_GPIO_ReadPin(GPIOC, Front_Infrared1);
    uint8_t s2 = HAL_GPIO_ReadPin(GPIOC, Front_Infrared2);
    char msg[32];
    sprintf(msg, "s1%ds2%d\n", s1 == GPIO_PIN_RESET ? 0 : 1, s2 == GPIO_PIN_RESET ? 0 : 1);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
