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
    int16_t ax, ay, az;  // 가속도
    int16_t gx, gy, gz;  // 자이로
} ImuRawData;

void MPU9250_Init(I2C_HandleTypeDef *hi2c);
ImuRawData MPU9250_ReadImu(I2C_HandleTypeDef *hi2c);

#endif /* INC_MPU9250_H_ */
