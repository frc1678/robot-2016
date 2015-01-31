#include "WPILib.h"

#ifndef PINCHERSYSTEM_H
#define PINCHERSYSTEM_H

class PincherSystem {

	VictorSP *rightToteAccel;
	VictorSP *leftToteAccel;
	VictorSP *rightRollers;
	VictorSP *leftRollers;


	DoubleSolenoid *openPinchers;


	bool pinchersOpen;

	bool rightOpen;
	bool leftOpen;

public:
	AnalogInput *bottomSensor;
	AnalogInput *topSensor;

	PincherSystem();

	~PincherSystem();

	void OpenPinchers();
	void ClosePinchers();
	void TogglePinchers();
	void StopPinchers();

	void OpenRight();
	void OpenLeft();

	bool BottomProximityTriggered();
	bool TopProximityTriggered();
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
