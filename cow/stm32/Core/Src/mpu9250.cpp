/*
 * mpu9250.cpp
 *
 *  Created on: Jun 4, 2025
 *      Author: kimyw
 */

#include "mpu9250.h"

void MPU9250_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // 장치 깨우기
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x6B, 1, &data, 1, 100);

    // 가속도 범위 설정 (±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x1C, 1, &data, 1, 100);

    // 자이로 범위 설정 (±250°/s)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x1B, 1, &data, 1, 100);
}

AccelRawData MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c) {
    uint8_t acc_data[6];
    AccelRawData accel;

    // MPU9250_ADDR는 8비트 주소로 <<1 되어야 함 (HAL 기준)
    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, 0x3B, 1, acc_data, 6, 100) == HAL_OK) {
        accel.ax = (int16_t)(acc_data[0] << 8 | acc_data[1]);
        accel.ay = (int16_t)(acc_data[2] << 8 | acc_data[3]);
        accel.az = (int16_t)(acc_data[4] << 8 | acc_data[5]);
    } else {
        // 읽기 실패 시 0으로 초기화 (또는 에러 처리 방식 선택)
        accel.ax = 0;
        accel.ay = 0;
        accel.az = 0;
    }

    return accel;
}
