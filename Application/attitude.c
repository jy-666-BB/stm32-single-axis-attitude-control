#include "attitude.h"
#include "math.h"
#include <stdio.h>

void Attitude_Update(
    MPU6050_Data_t *MPU6050,
    Attitude_t *Attitude,
    float dt
)
{

    float AccRoll;
    float AccPitch;

    AccRoll = atan2(MPU6050->AccY,MPU6050->AccZ)
    *180/3.1415926;
	
    AccPitch =
    atan2(-MPU6050->AccX,
    sqrt(MPU6050->AccY*MPU6050->AccY+MPU6050->AccZ*MPU6050->AccZ))
    *180/3.1415926;
	
    Attitude->Roll =0.98f*(Attitude->Roll+MPU6050->GyroX*dt)
    + 0.02f*AccRoll;

    Attitude->Pitch =0.98f*(Attitude->Pitch+MPU6050->GyroY*dt)
    + 0.02f*AccPitch;
}

