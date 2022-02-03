
#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float Ek;
	float Ek1;
	float EkSum;
}PID_typedef;

void PID_Init(PID_typedef * PID, float Kp, float Ki, float Kd);
float PID_Position(float targetDist, float actualDist, PID_typedef * PID);
int PID_Duty(float targetSpeed, float actualSpeed, PID_typedef * PID);

#endif
