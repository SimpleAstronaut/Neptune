/****************************************************************************
 *  Neptune pid module
 *
 *  pid¼ÆËãÄ£¿é
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/

#include "stdlib.h"
#include "pid.h"


void PID_Init(PID_t* Pid, float Kp, float Ki, float Kd, float integral_max, float outputmax, float errormax) {
    Pid->Kp = Kp;
    Pid->Ki = Ki;   
    Pid->Kd = Kd;
    Pid->integral = 0.0;
    Pid->last_error = 0.0;
    Pid->integral_max = integral_max;
    Pid->outputmax = outputmax;
    Pid->errormax = errormax;
}

int PID_General_Cal(PID_t* Pid, float target, float current) {
    float error = target - current;
    LimitMax(error, Pid->errormax);

    //(target >= 0 &&   target < current) || (target < 0 && target > current)
    //(error * Pid->last_error) < 0
    if ((target >= 0 && target < current) || (target < 0 && target > current))
    {
        Pid->integral = 0;
    }
    
    Pid->integral += error;
    //LimitMax(Pid->integral, Pid->integral_max);
    //LimitMax(Pid->integral, Pid->integral_max);
    if (abs(Pid->integral) > abs(Pid->integral_max) && Pid->integral_max != 0)
    {
        Pid->integral = Pid->integral_max;
    }

    float derivative = error - Pid->last_error;
    int output = (int)(Pid->Kp * error + Pid->Ki * Pid->integral + Pid->Kd * derivative);
    Pid->last_error = error;

    //LimitMax(output, Pid->outputmax);
    /*if (abs(output) > Pid->outputmax && Pid->outputmax != 0)
    {
        output = Pid->outputmax;
    }*/

    LimitMax(output, Pid->outputmax)
    return output;
}

int POSPID_General_Cal(PID_t* Pid, float target, float current)  
{
		if(target - current < -180)
		{
				target += 360;
		}
		if(target - current > 180)
		{
				target -= 360;
		}
		float error = target - current;
    LimitMax(error, Pid->errormax);

    //(target >= 0 &&   target < current) || (target < 0 && target > current)
    //(error * Pid->last_error) < 0
    if ((target >= 0 && target < current) || (target < 0 && target > current))
    {
        Pid->integral = 0;
    }
    
    Pid->integral += error;
    //LimitMax(Pid->integral, Pid->integral_max);
    //LimitMax(Pid->integral, Pid->integral_max);
    if (abs(Pid->integral) > abs(Pid->integral_max) && Pid->integral_max != 0)
    {
        Pid->integral = Pid->integral_max;
    }

    float derivative = error - Pid->last_error;
    int output = (int)(Pid->Kp * error + Pid->Ki * Pid->integral + Pid->Kd * derivative);
    Pid->last_error = error;

    //LimitMax(output, Pid->outputmax);
    /*if (abs(output) > Pid->outputmax && Pid->outputmax != 0)
    {
        output = Pid->outputmax;
    }*/

    LimitMax(output, Pid->outputmax)
    return output;
}
