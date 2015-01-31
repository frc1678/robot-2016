#include "WPILib.h"


#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H



class ControlLoop {

	Timer *PIDTimer;

	float integral;
	float lastErr;

	//flaot kp is public now
	float ki;
	float kd;



public:

	float kp;// has to be public due to logging
	ControlLoop (float inKP, float inKI, float inKD);

	~ControlLoop ();

	void StartLoop ();

	void ResetLoop ();

	void Stop();

	// Call this once every loop
	float CalibrateLoop (float err);


};



#endif
