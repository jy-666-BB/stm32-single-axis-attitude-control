#ifndef __MPU6050_H
#define __MPU6050_H


#include "stdint.h"


#define MPU6050_ADDR 0x68

typedef struct
{

    //原始数据
    int16_t AccX_Raw;
    int16_t AccY_Raw;
    int16_t AccZ_Raw;

    int16_t GyroX_Raw;
    int16_t GyroY_Raw;
    int16_t GyroZ_Raw;
	
    //物理量

    float AccX;
    float AccY;
    float AccZ;

    float GyroX;
    float GyroY;
    float GyroZ;

}MPU6050_Data_t;

extern int16_t GyroX_Bias;
extern int16_t GyroY_Bias;
extern int16_t GyroZ_Bias;

void MPU6050_Init(void);

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);

uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_GetData(MPU6050_Data_t *Data);

void MPU6050_Calibrate(void);
#endif
