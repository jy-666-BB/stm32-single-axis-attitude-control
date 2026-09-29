#include "pwm.h"
#include "tim.h"


void PWM_Start(void)
{

    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

}



void Servo_SetAngle(uint8_t angle)
{

    uint16_t compare;


    compare = 500 + (uint16_t)angle * 2000 / 180;


    __HAL_TIM_SET_COMPARE(&htim2,
                          TIM_CHANNEL_1,
                          compare);

}
