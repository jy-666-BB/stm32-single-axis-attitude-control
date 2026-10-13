#include "MPU6050.h"
#include "i2c.h"
#include <stdio.h>

/* I2C 操作超时时间，单位：毫秒 */
#define MPU6050_I2C_TIMEOUT_MS 10U

/* 陀螺仪三轴静态零偏 */
int16_t GyroX_Bias = 0;
int16_t GyroY_Bias = 0;
int16_t GyroZ_Bias = 0;

/* 加速度计三轴静态零偏 */
int16_t AccX_Bias = 0;
int16_t AccY_Bias = 0;
int16_t AccZ_Bias = 0;

/* 向 MPU6050 指定寄存器写入一个字节 */
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR << 1,
                             RegAddress,
                             I2C_MEMADD_SIZE_8BIT,
                             &Data,
                             1,
                             MPU6050_I2C_TIMEOUT_MS);
}

/* 从 MPU6050 指定寄存器开始连续读取若干个字节 */
static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t RegAddress, uint8_t *Data, uint16_t Size)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR << 1,
                            RegAddress,
                            I2C_MEMADD_SIZE_8BIT,
                            Data,
                            Size,
                            MPU6050_I2C_TIMEOUT_MS);
}

/* 初始化 MPU6050：检查设备 ID、唤醒芯片并配置采样参数 */
HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t who_am_i = 0;

    /* 需要依次配置的寄存器：采样率、低通滤波、陀螺仪量程、加速度计量程  */
    const uint8_t regs[] = {0x19, 0x1A, 0x1B, 0x1C};
    const uint8_t vals[] = {0x07, 0x03, 0x00, 0x00};

    /* 先读取 WHO_AM_I，确认总线上连接的是 MPU6050 */
    if (MPU6050_ReadReg(MPU6050_WHO_AM_I_REG, &who_am_i, 1) != HAL_OK ||
        who_am_i != MPU6050_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    /* 清除睡眠位，使 MPU6050 进入正常工作模式 */
    if (MPU6050_WriteReg(0x6B, 0x00) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 等待芯片内部时钟和传感器稳定 */
    HAL_Delay(100);

    /* 写入采样率、滤波和量程配置，任意一次失败都停止初始化 */
    for (uint8_t i = 0; i < 4; i++)
    {
        if (MPU6050_WriteReg(regs[i], vals[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/* 从 0x3B 开始连续读取 14 字节，并转换为加速度和角速度 */
HAL_StatusTypeDef MPU6050_GetData(MPU6050_Data_t *Data)
{
    uint8_t buffer[14];

    /* 读取加速度、温度和陀螺仪的原始数据 */
    if (MPU6050_ReadReg(0x3B, buffer, 14) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 加速度计数据：buffer[0..5]，每个轴由高字节和低字节组成 */
    Data->AccX_Raw = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    Data->AccY_Raw = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    Data->AccZ_Raw = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    /* 温度数据位于 buffer[6..7]，当前项目暂不使用 */
    /* 陀螺仪数据：buffer[8..13] */
    Data->GyroX_Raw = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    Data->GyroY_Raw = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    Data->GyroZ_Raw = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    /* 减去零偏并换算为物理量：加速度单位为 g，角速度单位为度/秒 */
    Data->AccX = (Data->AccX_Raw - AccX_Bias) / 16384.0f;
    Data->AccY = (Data->AccY_Raw - AccY_Bias) / 16384.0f;
    Data->AccZ = (Data->AccZ_Raw - AccZ_Bias) / 16384.0f;

    Data->GyroX = (Data->GyroX_Raw - GyroX_Bias) / 131.0f;
    Data->GyroY = (Data->GyroY_Raw - GyroY_Bias) / 131.0f;
    Data->GyroZ = (Data->GyroZ_Raw - GyroZ_Bias) / 131.0f;

    return HAL_OK;
}

/* 采集静止状态下的数据，计算加速度计和陀螺仪零偏 */
HAL_StatusTypeDef MPU6050_Calibrate(void)
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

    /* 多次采样并累加，只统计读取成功的数据 */
    for (int i = 0; i < 1000; i++)
    {
        if (MPU6050_GetData(&data) == HAL_OK)
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

    /* 没有任何有效数据时，标定失败，避免除零和无效零偏 */
    if (valid_samples == 0)
    {
        printf("Calibration failed: no valid samples\r\n");
        return HAL_ERROR;
    }

    /* 陀螺仪静止时理论输出为 0，用平均原始值作为零偏 */
    GyroX_Bias = GyroX_sum / valid_samples;
    GyroY_Bias = GyroY_sum / valid_samples;
    GyroZ_Bias = GyroZ_sum / valid_samples;

    /* 加速度计 X/Y 静止时约为 0 g，Z 轴约为 1 g */
    AccX_Bias = AccX_sum / valid_samples;
    AccY_Bias = AccY_sum / valid_samples;
    AccZ_Bias = AccZ_sum / valid_samples - 16384;

    printf("Gyro Bias:\r\n");
    printf("%d %d %d\r\n", GyroX_Bias, GyroY_Bias, GyroZ_Bias);

    printf("Acc Bias:\r\n");
    printf("%d %d %d\r\n", AccX_Bias, AccY_Bias, AccZ_Bias);

    return HAL_OK;
}
