#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "mpu6050.h"

typedef struct
{

    float Roll;
    float Pitch;

}Attitude_t;

extern Attitude_t Attitude;

void Attitude_Update(
    MPU6050_Data_t *MPU6050,
    Attitude_t *Attitude,
    float dt
);


#endif
