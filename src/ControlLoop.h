#include "WPILib.h"


#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H



class ControlLoop {

	Timer *PIDTimer;

	float integral;
	float lastErr;

	float kp;
	float ki;
	float kd;



public:

	ControlLoop (float inKP, float inKI, float inKD);

	~ControlLoop ();

	void StartLoop ();

	void ResetLoop ();

	void Stop();

	// Call this once every loop
	float CalibrateLoop (float err);


};



#endif
