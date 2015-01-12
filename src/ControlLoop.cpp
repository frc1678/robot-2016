#include "ControlLoop.h"

ControlLoop::ControlLoop(float inKP, float inKI, float inKD) {

	PIDTimer = new Timer();

	integral = 0.0;
	lastErr = 0.0;

	kp = inKP;
	ki = inKI;
	kd = inKD;

}

ControlLoop::~ControlLoop() { /* Deconstructor*/
}

void ControlLoop::StartLoop() {
	PIDTimer->Reset();
	PIDTimer->Start();

}

void ControlLoop::ResetLoop() {
	PIDTimer->Reset();
}

void ControlLoop::Stop() {
	PIDTimer->Stop();
}

// Call this once every loop
float ControlLoop::CalibrateLoop(float err) {

	// Find the difference between where you want to end and where you are
	// err = fabs(target - input);
	// Calculate how much time has past
	float dt = PIDTimer->Get();
	// Reset the time to zero
	PIDTimer->Reset();
	// Calculate the integral
	integral += err * dt;
	// Calculate the derivative (not in use)
	float der = (err + lastErr) / dt;
	// Reset the error
	lastErr = err;

	// PID coefficients for driving straight, but NOT turning
	//MV output
	return kp * err + ki * integral + kd * der;
	//return 0.05*err + 0.005*integral + 0.00*der;
}
