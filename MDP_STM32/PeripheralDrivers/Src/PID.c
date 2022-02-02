#include "PID.h"

void PID_Init(PID_typedef * PID, float Kp, float Ki, float Kd) {
	PID->Kp = Kp;
	PID->Kp = Ki;
	PID->Kp = Kd;
}

float PID_Position() {

}

float PID_Speed(float targetSpeed, float actualSpeed, PID_typedef * PID) {
	PID->Ek = targetSpeed - actualSpeed;
	PID->EkSum += PID->Ek;
	PID->Ek1 = PID->Ek;

	return PID->Kp * PID->Ek + PID->Ki * PID->EkSum + PID->Kd * (PID->Ek1 - PID->Ek);
}
