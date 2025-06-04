/*
 * mpu9250.h
 *
 *  Created on: Jun 4, 2025
 *      Author: kimyw
 */

#ifndef INC_MPU9250_H_
#include "stm32f4xx_hal.h"
#define INC_MPU9250_H_

#define MPU9250_ADDR 0x68 << 1
typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
} AccelRawData;

void MPU9250_Init(I2C_HandleTypeDef *hi2c);
AccelRawData MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c);

#endif /* INC_MPU9250_H_ */
