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

ImuRawData MPU9250_ReadImu(I2C_HandleTypeDef *hi2c) {
    uint8_t raw_data[14];  // accel(6) + temp(2) + gyro(6)
    ImuRawData imu = {0};

    // MPU9250_ADDR는 HAL에서는 8비트 주소 (보통 0x68 << 1)
    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, 0x3B, 1, raw_data, 14, 100) == HAL_OK) {
        // 가속도
        imu.ax = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        imu.ay = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        imu.az = (int16_t)(raw_data[4] << 8 | raw_data[5]);

        // 자이로
        imu.gx = (int16_t)(raw_data[8] << 8 | raw_data[9]);
        imu.gy = (int16_t)(raw_data[10] << 8 | raw_data[11]);
        imu.gz = (int16_t)(raw_data[12] << 8 | raw_data[13]);
    }

    return imu;
}
