/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "event_groups.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "tim.h"
#include "usart.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
osSemaphoreId_t imuMutexHandle;
const osSemaphoreAttr_t imuMutex_attributes = { .name = "imuMutex" };
typedef struct {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t mx, my, mz;
	float temperature;
	float pitch, roll, yaw; // 추가
} ImuRawData;

ImuRawData sharedImuData;
QueueHandle_t MotorSpeedQueue;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for monitortingTask */
osThreadId_t monitortingTaskHandle;
const osThreadAttr_t monitortingTask_attributes = {
  .name = "monitortingTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for navigationTask */
osThreadId_t navigationTaskHandle;
const osThreadAttr_t navigationTask_attributes = {
  .name = "navigationTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void parseCommand(char *cmd);
void vmonitoring(void *argument);
void vnavigation(void *argument);
/* USER CODE END FunctionPrototypes */

void vmonitoring(void *argument);
void vnavigation(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	imuMutexHandle = osSemaphoreNew(1, 1, &imuMutex_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	MotorSpeedQueue = xQueueCreate(64, sizeof(CommandMessage));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of monitortingTask */
  monitortingTaskHandle = osThreadNew(vmonitoring, NULL, &monitortingTask_attributes);

  /* creation of navigationTask */
  navigationTaskHandle = osThreadNew(vnavigation, NULL, &navigationTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_vmonitoring */
/**
 * @brief  Function implementing the monitortingTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_vmonitoring */
void vmonitoring(void *argument)
{
  /* USER CODE BEGIN vmonitoring */
	char tx_buffer[256];
	for (;;) {
		int32_t left_encoder = readEncoder(&htim3);
		int32_t right_encoder = readEncoder(&htim4);

		updateImuData();  // BSP 기반 값으로 업데이트

		extern int current_left_pwm;
		extern int current_right_pwm;

		int trash_state = 0;
		int emergency_state = 0;

		snprintf(tx_buffer, sizeof(tx_buffer),
				"SPEED:L%d,R%d;TRASH:%d;EMERGENCY:%d;ENCODER:L%d,R%d;ACC:%d,%d,%d,GYRO:%d,%d,%d\n",
				current_left_pwm, current_right_pwm, trash_state,
				emergency_state, left_encoder, right_encoder, sharedImuData.ax,
				sharedImuData.ay, sharedImuData.az, sharedImuData.gx,
				sharedImuData.gy, sharedImuData.gz);

		HAL_UART_Transmit(&huart4, (uint8_t*) tx_buffer, strlen(tx_buffer),
		HAL_MAX_DELAY);

		osDelay(500);  // 500ms마다 송신
	}
  /* USER CODE END vmonitoring */
}

/* USER CODE BEGIN Header_vnavigation */
/**
 * @brief Function implementing the navigationTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vnavigation */
void vnavigation(void *argument)
{
  /* USER CODE BEGIN vnavigation */
	/* Infinite loop */
	CommandMessage msg;
	for (;;) {
		if (xQueueReceive(MotorSpeedQueue, &msg, 1) == pdTRUE) {
			parseCommand(msg.cmd);  // 예: "L100R120"
		}

		osDelay(100);
	}
  /* USER CODE END vnavigation */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void parseCommand(char *cmd) {
	int left_pwm = 0, right_pwm = 0;

	char *l_ptr = strchr(cmd, 'L');
	char *r_ptr = strchr(cmd, 'R');

	if (l_ptr && r_ptr) {
		left_pwm = atoi(l_ptr + 1);   // 'L' 다음부터 정수로 변환
		right_pwm = atoi(r_ptr + 1);  // 'R' 다음부터 정수로 변환

		// PWM 제한 범위 적용
		if (left_pwm > 255)
			left_pwm = 255;
		if (left_pwm < -255)
			left_pwm = -255;
		if (right_pwm > 255)
			right_pwm = 255;
		if (right_pwm < -255)
			right_pwm = -255;

		if (left_pwm > 0 && right_pwm > 0) {
			setMotorMode(FORWARD);

		} else if (left_pwm < 0 && right_pwm < 0) {
			setMotorMode(BACKWARD);

		} else if (left_pwm > 0 && right_pwm < 0) {
			setMotorMode(ROTATE_RIGHT);

		} else if (left_pwm < 0 && right_pwm > 0) {
			setMotorMode(ROTATE_LEFT);

		} else {
			setMotorMode(STOP);
		}

		setMotorSpeed('L', left_pwm);
		setMotorSpeed('R', right_pwm);
	} else {
		osDelay(1);
	}
}

void updateImuData() {
    int16_t acc[3];
    float   gyro_f[3];

    /* 1) 센서 읽기 -------------------------------------------------------- */
    BSP_ACCELERO_GetXYZ(acc);        // ±2 g, raw LSB
    BSP_GYRO_GetXYZ  (gyro_f);       // °/s,   float

    /* 2) 공유 구조체 보호 -------------------------------------------------- */
    osSemaphoreAcquire(imuMutexHandle, osWaitForever);

    sharedImuData.ax = acc[0];
    sharedImuData.ay = acc[1];
    sharedImuData.az = acc[2];

    sharedImuData.gx = (int16_t)gyro_f[0];
    sharedImuData.gy = (int16_t)gyro_f[1];
    sharedImuData.gz = (int16_t)gyro_f[2];

    osSemaphoreRelease(imuMutexHandle);
}
/* USER CODE END Application */

