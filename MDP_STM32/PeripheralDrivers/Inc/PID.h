
#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float Ek;
	float Ek1;
	float Ek2;
	float EkSum;
}PID_typedef;

void PID_Init(PID_typedef * PID, float Kp, float Ki, float Kd);
float PID_Position();
float PID_Speed(float targetSpeed, float actualSpeed, PID_typedef * PID);

#endif
