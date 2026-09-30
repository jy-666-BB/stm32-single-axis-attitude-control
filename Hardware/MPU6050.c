#include "MPU6050.h"
#include "i2c.h"
#include <stdio.h>

int16_t GyroX_Bias = 0;
int16_t GyroY_Bias = 0;
int16_t GyroZ_Bias = 0;

int16_t AccX_Bias = 0;
int16_t AccY_Bias = 0;
int16_t AccZ_Bias = 0;

void MPU6050_Init(void)
{
    // 1. 唤醒 MPU6050
    MPU6050_WriteReg(0x6B, 0x00);
    HAL_Delay(100);

    // 2. 设置采样率分频
    MPU6050_WriteReg(0x19, 0x07);

    // 3. 设置低通滤波
    MPU6050_WriteReg(0x1A, 0x03);

    // 4. 设置陀螺仪量程
    MPU6050_WriteReg(0x1B, 0x00);

    // 5. 设置加速度计量程
    MPU6050_WriteReg(0x1C, 0x00);
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR << 1,
                      RegAddress,
                      I2C_MEMADD_SIZE_8BIT,
                      &Data,
                      1,
                      100);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0;

    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR << 1,
                     RegAddress,
                     I2C_MEMADD_SIZE_8BIT,
                     &Data,
                     1,
                     100);

    return Data;
}

MPU6050_Status_t MPU6050_GetData(MPU6050_Data_t *Data)
{
    uint8_t buffer[14];

    // 从 0x3B 开始连续读取加速度、温度和陀螺仪数据
    if (HAL_I2C_Mem_Read(&hi2c1,
                         MPU6050_ADDR << 1,
                         0x3B,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         14,
                         10) != HAL_OK)
    {
        return MPU6050_ERROR_I2C;
    }

    Data->AccX_Raw = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    Data->AccY_Raw = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    Data->AccZ_Raw = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    Data->GyroX_Raw = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    Data->GyroY_Raw = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    Data->GyroZ_Raw = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    // 原始数据转换为物理量
    Data->AccX = (Data->AccX_Raw - AccX_Bias) / 16384.0f;
    Data->AccY = (Data->AccY_Raw - AccY_Bias) / 16384.0f;
    Data->AccZ = (Data->AccZ_Raw - AccZ_Bias) / 16384.0f;

    Data->GyroX = (Data->GyroX_Raw - GyroX_Bias) / 131.0f;
    Data->GyroY = (Data->GyroY_Raw - GyroY_Bias) / 131.0f;
    Data->GyroZ = (Data->GyroZ_Raw - GyroZ_Bias) / 131.0f;

    return MPU6050_OK;
}

void MPU6050_Calibrate(void)
{
    int32_t GyroX_sum = 0;
    int32_t GyroY_sum = 0;
    int32_t GyroZ_sum = 0;

    int32_t AccX_sum = 0;
    int32_t AccY_sum = 0;
    int32_t AccZ_sum = 0;

    uint16_t valid_samples = 0;
    MPU6050_Data_t data;

    printf("Calibrating...\r\n");

    for (int i = 0; i < 1000; i++)
    {
        if (MPU6050_GetData(&data) == MPU6050_OK)
        {
            GyroX_sum += data.GyroX_Raw;
            GyroY_sum += data.GyroY_Raw;
            GyroZ_sum += data.GyroZ_Raw;

            AccX_sum += data.AccX_Raw;
            AccY_sum += data.AccY_Raw;
            AccZ_sum += data.AccZ_Raw;

            valid_samples++;
        }

        HAL_Delay(5);
    }

    if (valid_samples == 0)
    {
        printf("Calibration failed: no valid samples\r\n");
        return;
    }

    GyroX_Bias = GyroX_sum / valid_samples;
    GyroY_Bias = GyroY_sum / valid_samples;
    GyroZ_Bias = GyroZ_sum / valid_samples;

    AccX_Bias = AccX_sum / valid_samples;
    AccY_Bias = AccY_sum / valid_samples;
    AccZ_Bias = AccZ_sum / valid_samples - 16384;

    printf("Gyro Bias:\r\n");
    printf("%d %d %d\r\n", GyroX_Bias, GyroY_Bias, GyroZ_Bias);

    printf("Acc Bias:\r\n");
    printf("%d %d %d\r\n", AccX_Bias, AccY_Bias, AccZ_Bias);
}