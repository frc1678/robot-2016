/*
 * SteeringWheelDrive.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <teleop/SteeringWheelDrive.h>
#define MIN(a, b) (a < b ? a : b)
#define ABS(a) (a > 0 ? a : -a)

float deadzone(float val, float thresh) {
	if (ABS(val) < thresh) {
		val = 0;
	} else {
		if (val < 0) {
			val += thresh;
		} else {
			val -= thresh;
		}
		val /= (1 - thresh);
	}
	return val;
}

SteeringWheelDrive::SteeringWheelDrive(RobotDrive* drive, Joystick* wheel,
		Joystick* speed, ConstantsLoader *k) {
	// TODO Auto-generated constructor stub
	driveTrain = drive;
	steeringWheel = wheel;
	speedController = speed;
	deadZoneSteering=k->getConstant("KSTEERINGDEADZONE", .1);
	deadZoneSpeed=k->getConstant("KSPEEDDEADZONE", .5);
}

SteeringWheelDrive::~SteeringWheelDrive() {
	// TODO Auto-generated destructor stub
}

void SteeringWheelDrive::drive(int mode) {

	float speed = speedController->GetY();
	speed = deadzone(speed, .05);
	float steeringVal = steeringWheel->GetX();
	steeringVal = deadzone(steeringVal, .1);
	steeringVal *= (-speedController->GetZ()) / 2 + 1.2;
	float lCoeff = 1 + steeringVal;
	float rCoeff = 1 - steeringVal;

	switch (mode) {
	case 0:
		break;//everything already set up
	case 1:
		speed = -.5;
		if (steeringVal < 0) {
			rCoeff -= .15;
			lCoeff = -rCoeff;
		} else if (steeringVal > 0) {
			lCoeff -= .15;
			rCoeff = -lCoeff;
		} else {
			rCoeff = 0;
			lCoeff = 0;
		}
		break;
	}

	driveTrain->TankDrive(lCoeff * speed, rCoeff * speed);
}

