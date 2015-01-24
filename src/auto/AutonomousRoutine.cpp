/*
 * AutonomousRoutine.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <auto/AutonomousRoutine.h>

AutonomousRoutine::AutonomousRoutine(Solenoid* stingerPin,
		Solenoid* releaseTension, RobotDrive* drive,
		ConstantsLoader* autoKLoad, Victor* winch) {
	this->drive = drive;
	this->pin = stingerPin;
	this->tension = releaseTension;
	this->stingerTime = autoKLoad->getConstant("KSTINGERTIME", 300);
	this->moveBeforeTensionDropTime = autoKLoad->getConstant("KMOVEBEFORETIME",
			1000);
	this->moveAfterTensionDropTime = autoKLoad->getConstant("KMOVEAFTERTIME",
			1500);
	this->foldTime = autoKLoad->getConstant("KFOLDTIME", 5000);
	autoTimer = new Timer();
	this->v = winch;
}

AutonomousRoutine::~AutonomousRoutine() {
	// TODO Auto-generated destructor stub
}

void AutonomousRoutine::start() {
	pin->Set(true); // Release the stinger ASAP
	autoTimer->Start();
	while (autoTimer->Get() < stingerTime / 1000) {
		delayMillis(5);
	}
	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	while (autoTimer->Get() < moveBeforeTensionDropTime / 1000) {
		// once we get the recycling containers, bring them back to our side
		drive->TankDrive(-1, -1);
		delayMillis(5);
	}
	// Drop the tension
	tension->Set(true);
	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();

	while (autoTimer->Get() < moveAfterTensionDropTime / 1000) {
		//get to the auto zone
		drive->TankDrive(-1, -1);
		delayMillis(5);
	}

	//stop once we are in the auto zone
	drive->TankDrive(0.0, 0.0);
	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	while (autoTimer->Get() < foldTime / 1000) {
		// TODO Find out how to use the winch
		delayMillis(5);
	}
}
