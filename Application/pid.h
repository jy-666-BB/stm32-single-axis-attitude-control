#ifndef __PID_H
#define __PID_H

extern float P_Output;
extern float D_Output;

typedef struct
{

    //PID参数
    float Kp;
    float Ki;
    float Kd;

    //误差
    float Error;
    float Last_Error;

    //积分项
    float Integral;

    //输出
    float Output;

    //输出限制
    float Max_Output;

}PID_t;

//初始化PID
void PID_Init(
        PID_t *pid,
        float kp,
        float ki,
        float kd,
        float max_output
);

//计算PID输出
float PID_Calculate(
        PID_t *pid,
        float target,
        float measure,
        float dt
);
		
#endif
		