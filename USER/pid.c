#include "pid.h"

#define OutPut_UP_LIMIT 1800
#define OutPut_DW_LIMIT -1800

#define Interger_DW_LIMIT -10000
#define Interger_UP_LIMIT 10000

#define EncoderLine 2500

PIDController pid;  // 初始化PID控制器
volatile int recvPosiInitVal = 0;    										// 记录初始位移
int givenExecPosiVal = 0;    


void PIDController_Init(PIDController *pid) {

    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;

    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;

    // 输出阈值
    pid->limMin= OutPut_DW_LIMIT;
    pid->limMax= OutPut_UP_LIMIT;

    // 积分饱和上限
    pid->limMaxInt = Interger_UP_LIMIT;
    pid->limMinInt = Interger_DW_LIMIT;

    pid->T = 0.04;

    pid->Kp = 500;
    pid->Ki=  0.002;
    pid->Kd = 0;

    pid->out = 0.0f;

}	

float PIDController_Update(PIDController *pid, float setpoint, float measurement) 
{
    /*
    * Error signal
    */
    float error = setpoint - measurement;


    /*
    * Proportional
    */
    float proportional = pid->Kp * error;


    /*
    * Integral
    */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }


    /*
    * Derivative (band-limited differentiator)
    */

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


    /*
    * Compute output and apply limits
    */
    pid->out = proportional + pid->integrator + pid->differentiator;

    pid->out  = (pid->out/EncoderLine)/2; //转换为rpm

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}

void PIDController_Reset(PIDController *pid)
{
    pid->integrator = 0;
    pid->differentiator = 0;
    pid->prevMeasurement = 0;
    pid->prevError = 0;

    pid->out = 0.0f;
}
