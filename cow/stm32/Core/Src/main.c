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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "event_groups.h"
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  FORWARD,
  BACKWARD,
  ROTATE_RIGHT, // 오른방향 제자리 회전
  ROTATE_LEFT,  // 왼 방향 제자리 회전
  STOP
} DriveMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EmergencyOccure (1 << 0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
    .name = "CommTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for NavigationTask */
osThreadId_t NavigationTaskHandle;
const osThreadAttr_t NavigationTask_attributes = {
    .name = "NavigationTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for IntakeTask */
osThreadId_t IntakeTaskHandle;
const osThreadAttr_t IntakeTask_attributes = {
    .name = "IntakeTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for TrashBinMonitor */
osThreadId_t TrashBinMonitorHandle;
const osThreadAttr_t TrashBinMonitor_attributes = {
    .name = "TrashBinMonitor",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for LiftControlTask */
osThreadId_t LiftControlTaskHandle;
const osThreadAttr_t LiftControlTask_attributes = {
    .name = "LiftControlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for SystemMonitorTa */
osThreadId_t SystemMonitorTaHandle;
const osThreadAttr_t SystemMonitorTa_attributes = {
    .name = "SystemMonitorTa",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for EmergencyTask */
osThreadId_t EmergencyTaskHandle;
const osThreadAttr_t EmergencyTask_attributes = {
    .name = "EmergencyTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh7,
};
/* Definitions for Emergency */
osEventFlagsId_t EmergencyHandle;
const osEventFlagsAttr_t Emergency_attributes = {.name = "Emergency"};
/* USER CODE BEGIN PV */
EventGroupHandle_t emegencyEventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
// 기본 테스크
void StartDefaultTask(void *argument);
// 일반적인 작업들을 하는 task
void vCommTask(void *argument);
// 주행 알고리즘 관련 task
void vNavigationTask(void *argument);
// 쓰레기 수거 알고리즘 task
void vIntakeTask(void *argument);
// 쓰레기통 생태 체크 task
void vTrashBinMonitorTask(void *argument);
// 리프트 컨트롤 task
void vLiftControlTask(void *argument);
// 센서 값이나 데이터 모니터링 task
void vSystemMonitorTask(void *argument);
// 긴급 상황 task
void vEmergencyTask(void *argument);

/* USER CODE BEGIN PFP */
// 외부 긴급 버튼이 눌렸는지 확인하는 함수
bool is_emergency_btn_pressed();
void setMotorMode(DriveMode mode);
void setMotorSpeed(char motor_position, int speed);
void motorStart();
void motorShutdown();
void startEncoder(TIM_HandleTypeDef *htim);
int16_t readEncoder(TIM_HandleTypeDef *htim);
int16_t calRPM(char method, int8_t MT, int16_t encoder_count, float time,
               int8_t PPR, int8_t ratio);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_I2S3_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  emegencyEventGroup = xEventGroupCreate();
  motorStart();
  motorShutdown();
  setMotorMode(FORWARD);
  osDelay(3000);
  startEncoder(&htim3);
  startEncoder(&htim2);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  int8_t target_distance_cm = 100;
  int8_t PPR = 11;
  int8_t GEAR_RATIO = 30;
  float WHEEL_CIRCUMFERENCE_CM = 2 * 3.1415 * 3.5;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(vCommTask, NULL, &CommTask_attributes);

  /* creation of NavigationTask */
  NavigationTaskHandle =
      osThreadNew(vNavigationTask, NULL, &NavigationTask_attributes);

  /* creation of IntakeTask */
  IntakeTaskHandle = osThreadNew(vIntakeTask, NULL, &IntakeTask_attributes);

  /* creation of TrashBinMonitor */
  TrashBinMonitorHandle =
      osThreadNew(vTrashBinMonitorTask, NULL, &TrashBinMonitor_attributes);

  /* creation of LiftControlTask */
  LiftControlTaskHandle =
      osThreadNew(vLiftControlTask, NULL, &LiftControlTask_attributes);

  /* creation of SystemMonitorTa */
  SystemMonitorTaHandle =
      osThreadNew(vSystemMonitorTask, NULL, &SystemMonitorTa_attributes);

  /* creation of EmergencyTask */
  EmergencyTaskHandle =
      osThreadNew(vEmergencyTask, NULL, &EmergencyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Emergency */
  EmergencyHandle = osEventFlagsNew(&Emergency_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /*
   * Polarity가 Rising => 1체배 하지만 채널 두개를 사용 하므로 2체배
   */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD,
                    Left_Motor_IN1_Pin | Left_Motor_IN2_Pin |
                        Right_Motor_IN1_Pin | Right_Motor_IN2_Pin | LD4_Pin |
                        LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin External_Emegency_BTN_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin | External_Emegency_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_Motor_IN1_Pin Left_Motor_IN2_Pin
     Right_Motor_IN1_Pin Right_Motor_IN2_Pin LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = Left_Motor_IN1_Pin | Left_Motor_IN2_Pin |
                        Right_Motor_IN1_Pin | Right_Motor_IN2_Pin | LD4_Pin |
                        LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setMotorMode(DriveMode mode) {
  switch (mode) {
  case 0:
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN2_Pin, GPIO_PIN_RESET);
    break;
  case 1:
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN2_Pin, GPIO_PIN_SET);
    break;
    // 으론쪽으로 회전 => Left Motor 전진 , Right Motor 후진
  case 2:
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN2_Pin, GPIO_PIN_SET);
    break;
    // 왼쪽으로 회전 => Left Motor 후진 , Right Motor 전진
  case 3:
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN2_Pin, GPIO_PIN_RESET);
    break;
  case 4:
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Left_Motor_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Right_Motor_IN2_Pin, GPIO_PIN_RESET);
    break;
  }
}

void setMotorSpeed(char motor_position, int speed) {
  if (motor_position == 'L') {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed * 10);
  } else if (motor_position == 'R') {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed * 10);
  }
}

void motorStart() {
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void motorShutdown() {
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}

void startEncoder(TIM_HandleTypeDef *htim) {
  HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

int16_t readEncoder(TIM_HandleTypeDef *htim) {
  return (int16_t)__HAL_TIM_GET_COUNTER(htim);
}

/* MF = Multiplication Factor
 * method = M or T
 */
int16_t calRPM(char method, int8_t MT, int16_t encoder_count, float time,
               int8_t PPR, int8_t ratio) {
  int16_t result;
  float temp;
  if (method == 'M') {
    temp = (60 * encoder_count) / (time * ratio * PPR * MT);
    result = (int)temp;
  } else {
    // T Method
    temp = 60 / (time * PPR * MT);
    result = (int)temp;
  }
  return result;
}

bool is_emergency_btn_pressed() { return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15); }
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
    osDelay(500);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vCommTask */
/**
 * 일반적인 작업들을 하는 task
 * @brief Function implementing the CommTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vCommTask */
void vCommTask(void *argument) {
  /* USER CODE BEGIN vCommTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vCommTask */
}

/* USER CODE BEGIN Header_vNavigationTask */
/**
 * 주행 알고리즘 관련 task
 * @brief Function implementing the NavigationTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vNavigationTask */
void vNavigationTask(void *argument) {
  /* USER CODE BEGIN vNavigationTask */
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
    osDelay(500);
    setMotorSpeed('L', 50);
    setMotorSpeed('R', 50);
    EventBits_t bits = xEventGroupWaitBits(emegencyEventGroup, EmergencyOccure,
                                           pdTRUE,  // clear on exit
                                           pdFALSE, // wait for ANY
                                           0);

    if (bits & EmergencyOccure) {
      motorShutdown();
    }
    osDelay(1);
  }
  /* USER CODE END vNavigationTask */
}

/* USER CODE BEGIN Header_vIntakeTask */
/** 쓰레기 수거 알고리즘 task
 * @brief Function implementing the IntakeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vIntakeTask */
void vIntakeTask(void *argument) {
  /* USER CODE BEGIN vIntakeTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vIntakeTask */
}

/* USER CODE BEGIN Header_vTrashBinMonitorTask */
/** 쓰레기통 생태 체크 task
 * @brief Function implementing the TrashBinMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTrashBinMonitorTask */
void vTrashBinMonitorTask(void *argument) {
  /* USER CODE BEGIN vTrashBinMonitorTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vTrashBinMonitorTask */
}

/* USER CODE BEGIN Header_vLiftControlTask */
/**
 * 리프트 컨트롤 task
 * @brief Function implementing the LiftControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vLiftControlTask */
void vLiftControlTask(void *argument) {
  /* USER CODE BEGIN vLiftControlTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vLiftControlTask */
}

/* USER CODE BEGIN Header_vSystemMonitorTask */
/**
 * 센서 값이나 데이터 모니터링 task
 * @brief Function implementing the SystemMonitorTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vSystemMonitorTask */
void vSystemMonitorTask(void *argument) {
  /* USER CODE BEGIN vSystemMonitorTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vSystemMonitorTask */
}

/* USER CODE BEGIN Header_vEmergencyTask */
/**
 * 긴급 상황 task
 * @brief Function implementing the EmergencyTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vEmergencyTask */
void vEmergencyTask(void *argument) {
  /* USER CODE BEGIN vEmergencyTask */
  /* Infinite loop */
  for (;;) {
    if (is_emergency_btn_pressed()) {
      xEventGroupSetBits(emegencyEventGroup, EmergencyOccure);
    }
    osDelay(1);
  }
  /* USER CODE END vEmergencyTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
