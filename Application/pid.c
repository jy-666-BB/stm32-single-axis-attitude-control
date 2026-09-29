#include "pid.h"


void PID_Init(
        PID_t *pid,
        float kp,
        float ki,
        float kd,
        float max_output
)
{

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->Error = 0;

    pid->Last_Error = 0;

    pid->Integral = 0;

    pid->Output = 0;

    pid->Max_Output = max_output;

}


float PID_Calculate(
        PID_t *pid,
        float target,
        float measure,
        float dt
)
{

    //计算误差
    pid->Error = target - measure;

    //积分
    pid->Integral += pid->Error * dt;

    //微分
    float derivative =
        (pid->Error - pid->Last_Error) / dt;

    //PID输出
    pid->Output =
        pid->Kp * pid->Error
        +
        pid->Ki * pid->Integral
        +
        pid->Kd * derivative;

    //输出限幅
    if(pid->Output > pid->Max_Output)
    {
        pid->Output = pid->Max_Output;
    }

    if(pid->Output < -pid->Max_Output)
    {
        pid->Output = -pid->Max_Output;
    }

    //保存误差
    pid->Last_Error = pid->Error;

    return pid->Output;

}
