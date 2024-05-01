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

extern PIDController pid;                                      // PID������
extern u16 g_rpm_lastTwentyHistory[20]; 												//����ʱ�䴰�ڣ�����ƽ���ٶȣ�����
extern u8 g_rpm_winCnt; 																			//����ʱ�䴰�ڸ�ֵ����

extern volatile int recvPosiInitVal;    										// ��¼��ʼλ��
extern int givenExecPosiVal;                               // �˴�λ�û��˶��������λ�� pulse


#endif // PID_H
