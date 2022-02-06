
#ifndef INC_PID_H_
#define INC_PID_H_


#include <stdio.h>

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float Ek;
	float Ek1;
	float EkSum;
}PID_typedef;

void PID_Init(PID_typedef * PID, float const Kp, float const Ki, float const Kd);
float PID_Position(float targetDist, float actualDist, PID_typedef * PID);
int PID_Correction(int16_t targetVal, int16_t actualVal, PID_typedef * PID);
int PID_Duty(float targetSpeed, float actualSpeed, PID_typedef * PID);

#endif
