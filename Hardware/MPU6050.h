#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

/* MPU6050 的 7 位 I2C 地址 */
#define MPU6050_ADDR 0x68

/* WHO_AM_I 寄存器和正常返回值 */
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_WHO_AM_I_VALUE 0x68

/* MPU6050 的原始数据和转换后的物理量 */
typedef struct
{
    /* 三轴加速度原始值 */
    int16_t AccX_Raw;
    int16_t AccY_Raw;
    int16_t AccZ_Raw;

    /* 三轴角速度原始值 */
    int16_t GyroX_Raw;
    int16_t GyroY_Raw;
    int16_t GyroZ_Raw;

    /* 三轴加速度，单位：g */
    float AccX;
    float AccY;
    float AccZ;

    /* 三轴角速度，单位：度/秒 */
    float GyroX;
    float GyroY;
    float GyroZ;
} MPU6050_Data_t;

/* 三轴陀螺仪静态零偏 */
extern int16_t GyroX_Bias;
extern int16_t GyroY_Bias;
extern int16_t GyroZ_Bias;

/* 初始化、读取和标定接口，成功返回 HAL_OK，失败返回 HAL_ERROR */
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_GetData(MPU6050_Data_t *Data);
HAL_StatusTypeDef MPU6050_Calibrate(void);

#endif
