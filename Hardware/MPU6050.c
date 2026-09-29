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

    // 1. 唤醒MPU6050
    MPU6050_WriteReg(0x6B,0x00);
	HAL_Delay(100);
    // 2. 设置采样率分频
    MPU6050_WriteReg(0x19,0x07);

    // 3. 设置低通滤波
    MPU6050_WriteReg(0x1A,0x03);

    // 4. 设置陀螺仪量程
    MPU6050_WriteReg(0x1B,0x00);

    // 5. 设置加速度量程
    MPU6050_WriteReg(0x1C,0x00);

}

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{

    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR<<1,
                      RegAddress,
                      I2C_MEMADD_SIZE_8BIT,
                      &Data,
                      1,
                      100);

}


uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{

    uint8_t Data;

    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR<<1,
                     RegAddress,
                     I2C_MEMADD_SIZE_8BIT,
                     &Data,
                     1,
                     100);

    return Data;

}

void MPU6050_GetData(MPU6050_Data_t *Data)
{

    uint8_t DataH,DataL;

    // AccX_Raw
    DataH = MPU6050_ReadReg(0x3B);
    DataL = MPU6050_ReadReg(0x3C);

    Data->AccX_Raw = (DataH << 8) | DataL;

    // AccY_Raw
    DataH = MPU6050_ReadReg(0x3D);
    DataL = MPU6050_ReadReg(0x3E);

    Data->AccY_Raw = (DataH << 8) | DataL;

    // AccZ_Raw
    DataH = MPU6050_ReadReg(0x3F);
    DataL = MPU6050_ReadReg(0x40);

    Data->AccZ_Raw = (DataH << 8) | DataL;

    // GyroX_Raw
    DataH = MPU6050_ReadReg(0x43);
    DataL = MPU6050_ReadReg(0x44);

    Data->GyroX_Raw = ((DataH<<8)|DataL);

    // GyroY_Raw
    DataH = MPU6050_ReadReg(0x45);
    DataL = MPU6050_ReadReg(0x46);

    Data->GyroY_Raw = ((DataH<<8)|DataL);

    // GyroZ_Raw
    DataH = MPU6050_ReadReg(0x47);
    DataL = MPU6050_ReadReg(0x48);

    Data->GyroZ_Raw = ((DataH<<8)|DataL);

	//原始数据转换
    Data->AccX =(Data->AccX_Raw - AccX_Bias)/16384.0f;

	Data->AccY =(Data->AccY_Raw - AccY_Bias)/16384.0f;

	Data->AccZ =(Data->AccZ_Raw - AccZ_Bias)/16384.0f;

	Data->GyroX =(Data->GyroX_Raw - GyroX_Bias)/131.0f;

	Data->GyroY =(Data->GyroY_Raw - GyroY_Bias)/131.0f;

	Data->GyroZ =(Data->GyroZ_Raw - GyroZ_Bias)/131.0f;
}

void MPU6050_Calibrate(void)
{

    int32_t GyroX_sum = 0;
    int32_t GyroY_sum = 0;
    int32_t GyroZ_sum = 0;

	int32_t AccX_sum = 0;
    int32_t AccY_sum = 0;
    int32_t AccZ_sum = 0;
	
    MPU6050_Data_t data;

    printf("Calibrating...\r\n");

    for(int i=0;i<1000;i++)
    {

        MPU6050_GetData(&data);

        // 陀螺仪累加
        GyroX_sum += data.GyroX_Raw;
        GyroY_sum += data.GyroY_Raw;
        GyroZ_sum += data.GyroZ_Raw;

		// 加速度累加
        AccX_sum += data.AccX_Raw;
        AccY_sum += data.AccY_Raw;
        AccZ_sum += data.AccZ_Raw;
		
        HAL_Delay(5);

    }
	
	/*
       陀螺仪偏置
       静止时理论值为0
    */
	
    GyroX_Bias = GyroX_sum / 1000;
    GyroY_Bias = GyroY_sum / 1000;
    GyroZ_Bias = GyroZ_sum / 1000;

	/*
       加速度偏置

       X,Y:
       静止≈0g

       Z:
       静止≈1g
       所以减去16384
    */

    AccX_Bias = AccX_sum / 1000;

    AccY_Bias = AccY_sum / 1000;

    AccZ_Bias = AccZ_sum / 1000 - 16384;
	
    printf("Gyro Bias:\r\n");

    printf("%d %d %d\r\n",
			GyroX_Bias,
			GyroY_Bias,
			GyroZ_Bias);
	
	printf("Acc Bias:\r\n");

    printf("%d %d %d\r\n",
			AccX_Bias,
			AccY_Bias,
			AccZ_Bias);

}
