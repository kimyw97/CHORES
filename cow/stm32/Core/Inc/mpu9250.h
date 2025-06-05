#ifndef MPU9250_HAL_COMPATIBLE_H
#define MPU9250_HAL_COMPATIBLE_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

class MPU9250 {
public:
    MPU9250(I2C_HandleTypeDef* i2cHandle);

    void writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    uint8_t readByte(uint8_t devAddr, uint8_t regAddr);
    void readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t count, uint8_t* dest);

    void getAccelData(int16_t* dest);
    void getGyroData(int16_t* dest);
    void getMagData(int16_t* dest);
    int16_t getTempData();

    void initMPU9250();
    void initAK8963(float* destination);
    void calibrateMPU9250(float* gyroBias, float* accelBias);

   float getAres();
    float getGres();
    float getMres();

    void MadgwickQuaternionUpdate(float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float mx, float my, float mz);

    void MahonyQuaternionUpdate(float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float mx, float my, float mz);

    float aRes, gRes, mRes;
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float pitch, roll, yaw;
    float q[4];
    float eInt[3];
    float deltat;
    float beta = 0.1f;       // 필터 게인 (0.01 ~ 0.4, 일반적으로 0.1)
    float sampleFreq = 100.0f;

private:
    I2C_HandleTypeDef* _i2c;
};

#endif // MPU9250_HAL_COMPATIBLE_H
