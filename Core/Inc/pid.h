#ifndef __PID_H
#define __PID_H

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


//定义pid结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float integral_max;
    float outputmax;
    float errormax;
}PID_t;

//static PID_t pid1;

void PID_Init(PID_t* Pid, float Kp, float Ki, float Kd, float integral_max, float outputmax, float errormax);
int PID_General_Cal(PID_t* Pid, float target, float current);
int POSPID_General_Cal(PID_t* Pid, float target, float current) ;
#endif
