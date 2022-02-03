#include "PID.h"

void PID_Init(PID_typedef * PID, float Kp, float Ki, float Kd) {
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->Ek = 0;
	PID->Ek1 = 0;
	PID->EkSum = 0;
}

int PID_Duty(float targetSpeed, float actualSpeed, PID_typedef * PID) {
	float duty;
	PID->Ek = targetSpeed - actualSpeed;
	PID->EkSum += PID->Ek;
	duty = PID->Kp * PID->Ek + PID->Ki * PID->EkSum + PID->Kd * (PID->Ek1 - PID->Ek);
	PID->Ek1 = PID->Ek;

	return 7199 * duty / 10000;
}

float PID_Position(float targetDist, float actualDist, PID_typedef * PID) {
	float posDelta;
	PID->Ek = targetDist - actualDist;
	PID->EkSum += PID->Ek;
	posDelta = PID->Kp * PID->Ek + PID->Ki * PID->EkSum + PID->Kd * (PID->Ek1 - PID->Ek);
	PID->Ek1 = PID->Ek;
	return posDelta;
}
