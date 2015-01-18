#include "WPILib.h"

#ifndef PINCHERSYSTEM_H
#define PINCHERSYSTEM_H

class PincherSystem {

	VictorSP *rightToteAccel;
	VictorSP *leftToteAccel;
	VictorSP *rightRollers;
	VictorSP *leftRollers;


	Solenoid *rightPincher;
	Solenoid *leftPincher;

	AnalogInput *bottomSensor;

	bool pinchersOpen;
	bool proxyTriggered;

public:


	PincherSystem();

	~PincherSystem();

	void OpenPinchers();
	void ClosePinchers();
	void TogglePinchers();
	void StopPinchers();

	bool ProximityTriggered();
	bool OpenState();

	void RunToteAccel();
	void RunPinchers();
	void RunAt(float x);

	void ReverseToteAccel();
	void ReversePinchers();
	void ReversePinchersSlow();

	void HumanLoad();

};

#endif
