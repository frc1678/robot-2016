#include "WPILib.h"

#ifndef STEERINGWHEELDRIVE_H_
#define STEERINGWHEELDRIVE_H_

class SteeringWheelDrive {
public:
	SteeringWheelDrive(RobotDrive* drive, Joystick* wheel, Joystick* speed);
	virtual ~SteeringWheelDrive();
	void drive();
protected:
	RobotDrive* driveTrain;
	Joystick* steeringWheel;
	Joystick* speedController;
};

#endif /* STEERINGWHEELDRIVE_H_ */
