#include "mpu9250.h"
#include "cmsis_os.h"

#define MPU9250_ADDRESS     (0x68 << 1)
#define AK8963_ADDRESS      (0x0C << 1)
#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43
#define TEMP_OUT_H          0x41
#define AK8963_ST1          0x02
#define AK8963_XOUT_L       0x03
#define AK8963_CNTL         0x0A
#define AK8963_ASAX         0x10

MPU9250::MPU9250(I2C_HandleTypeDef* i2cHandle) : _i2c(i2cHandle) {
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    eInt[0] = eInt[1] = eInt[2] = 0.0f;
    deltat = 0.0f;
}

void MPU9250::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    HAL_I2C_Mem_Write(_i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

uint8_t MPU9250::readByte(uint8_t devAddr, uint8_t regAddr) {
    uint8_t data;
    HAL_I2C_Mem_Read(_i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

void MPU9250::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t count, uint8_t* dest) {
    HAL_I2C_Mem_Read(_i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, dest, count, HAL_MAX_DELAY);
}

void MPU9250::getAccelData(int16_t* dest) {
    uint8_t raw[6];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, raw);
    dest[0] = ((int16_t)raw[0] << 8) | raw[1];
    dest[1] = ((int16_t)raw[2] << 8) | raw[3];
    dest[2] = ((int16_t)raw[4] << 8) | raw[5];
}

void MPU9250::getGyroData(int16_t* dest) {
    uint8_t raw[6];
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, raw);
    dest[0] = ((int16_t)raw[0] << 8) | raw[1];
    dest[1] = ((int16_t)raw[2] << 8) | raw[3];
    dest[2] = ((int16_t)raw[4] << 8) | raw[5];
}

int16_t MPU9250::getTempData() {
    uint8_t raw[2];
    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, raw);
    return ((int16_t)raw[0] << 8) | raw[1];
}

void MPU9250::getMagData(int16_t* dest) {
    uint8_t raw[7];
    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, raw);
        if (!(raw[6] & 0x08)) {
            dest[0] = ((int16_t)raw[1] << 8) | raw[0];
            dest[1] = ((int16_t)raw[3] << 8) | raw[2];
            dest[2] = ((int16_t)raw[5] << 8) | raw[4];
        }
    }
}

float MPU9250::getAres() {
    return aRes;
}

float MPU9250::getGres() {
    return gRes;
}

float MPU9250::getMres() {
    return mRes;
}

void MPU9250::initMPU9250() {
    writeByte(MPU9250_ADDRESS, 0x6B, 0x00); // Wake up device
   osDelay(100);
    writeByte(MPU9250_ADDRESS, 0x1A, 0x03); // DLPF_CFG
    writeByte(MPU9250_ADDRESS, 0x1B, 0x18); // Gyro Full Scale ±2000dps
    writeByte(MPU9250_ADDRESS, 0x1C, 0x10); // Accel Full Scale ±8g

    gRes = 2000.0f / 32768.0f;
    aRes = 8.0f / 32768.0f;
}

void MPU9250::initAK8963(float* destination) {
    uint8_t rawData[3];

    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    osDelay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Fuse ROM access
    osDelay(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, rawData);

    destination[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;
    destination[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
    destination[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;

    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    osDelay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x16); // Continuous measurement mode 2
    osDelay(10);

    mRes = 4912.0f / 32760.0f;
}

void MPU9250::calibrateMPU9250(float* gyroBias, float* accelBias) {
    int32_t gyroSum[3] = {0}, accelSum[3] = {0};
    int16_t g[3], a[3];

    for (int i = 0; i < 100; i++) {
        getGyroData(g);
        getAccelData(a);
        for (int j = 0; j < 3; j++) {
            gyroSum[j] += g[j];
            accelSum[j] += a[j];
        }
        osDelay(10);
    }

    for (int i = 0; i < 3; i++) {
        gyroBias[i] = gyroSum[i] / 100.0f * gRes;
        accelBias[i] = accelSum[i] / 100.0f * aRes;
    }
}

void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az,
                                       float gx, float gy, float gz,
                                       float mx, float my, float mz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q1mx, _2q1my, _2q1mz, _2q2mx;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;

    // Gradient descent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
         + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
         + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az)
         + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
         + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
         + (_2bx * q4 - 4.0f * _2bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az)
         + (-4.0f * _2bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
         + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
         + (_2bx * q1 - 4.0f * _2bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay)
         + (-4.0f * _2bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
         + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
         + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Apply feedback step
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    q1 += qDot1 * (1.0f / sampleFreq);
    q2 += qDot2 * (1.0f / sampleFreq);
    q3 += qDot3 * (1.0f / sampleFreq);
    q4 += qDot4 * (1.0f / sampleFreq);

    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

    // Convert quaternion to Euler angles
    pitch = asinf(-2.0f * (q1 * q3 - q2 * q4)) * 180.0f / M_PI;
    roll = atan2f(2.0f * (q1 * q2 + q3 * q4), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
    yaw = atan2f(2.0f * (q1 * q4 + q2 * q3), 1.0f - 2.0f * (q3 * q3 + q4 * q4)) * 180.0f / M_PI;
}

