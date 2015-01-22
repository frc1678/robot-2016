#include "WPILib.h"

#ifndef STEERINGWHEELDRIVE_H_
#define STEERINGWHEELDRIVE_H_

#include <logs/ConstantsLoader.h>

class SteeringWheelDrive {
public:
	float deadZoneSteering, deadZoneSpeed;
	SteeringWheelDrive(RobotDrive* drive, Joystick* wheel, Joystick* speed, ConstantsLoader* k);
	virtual ~SteeringWheelDrive();
	void drive(int mode = 0);
protected:
	RobotDrive* driveTrain;
	Joystick* steeringWheel;
	Joystick* speedController;
};

#endif /* STEERINGWHEELDRIVE_H_ */
