#ifndef PID_H
#define PID_H

#include "sys.h"

typedef struct {
    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits */
    float limMinInt;
    float limMaxInt;

    /* Sample time (in seconds) */
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError;			/* Required for integrator */
    float differentiator;
    float prevMeasurement;		/* Required for differentiator */

    /* Controller output */
    float out;

}PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
void PIDController_Reset(PIDController *pid);

extern PIDController pid;                                      // PID控制器
extern u16 g_rpm_lastTwentyHistory[20]; 												//滑动时间窗口，计算平均速度：队列
extern u8 g_rpm_winCnt; 																			//滑动时间窗口赋值计数

extern volatile int recvPosiInitVal;    										// 记录初始位移
extern int givenExecPosiVal;                               // 此次位置环运动给定相对位移 pulse


#endif // PID_H
