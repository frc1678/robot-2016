/*
 * SteeringWheelDrive.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <teleop/SteeringWheelDrive.h>

SteeringWheelDrive::SteeringWheelDrive(RobotDrive* drive, Joystick* wheel, Joystick* speed) {
	// TODO Auto-generated constructor stub
	driveTrain=drive;
	steeringWheel=wheel;
	speedController=speed;
}

SteeringWheelDrive::~SteeringWheelDrive() {
	// TODO Auto-generated destructor stub
}

void SteeringWheelDrive::drive()
{

	float lCoeff = 1 + steeringWheel->GetX();
	float rCoeff = 1 - steeringWheel->GetX();

	float speed = speedController->GetY();
	//TODO: add a dead zone
	if (-.2 < speed && .2 > speed && (steeringWheel->GetX() < -.15 || steeringWheel->GetX() > .15))
	{
		speed = -.5;
		if (steeringWheel->GetX() < 0)
		{
			rCoeff -= .15;
			lCoeff = -rCoeff;
		}
		else if (steeringWheel->GetX() > 0)
		{
			lCoeff -= .15;
			rCoeff = -lCoeff;
		}
		else
		{
			rCoeff = 0;
			lCoeff = 0;
		}
	}
	driveTrain->TankDrive(lCoeff * speed, rCoeff * speed);
}

