/*
 * AutonomousRoutine.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <auto/AutonomousRoutine.h>

AutonomousRoutine::AutonomousRoutine(Solenoid* stingerPin,
		Solenoid* disengage, RobotDrive* drive,
		ConstantsLoader* autoKLoad, Victor* winch) {
	this->drive = drive;
	this->pin = stingerPin;
	this->disengageHooks = disengage;
	this->stingerTime = autoKLoad->getConstant("KSTINGERTIME", 300);
	this->moveBeforeTensionDrop = autoKLoad->getConstant("KMOVEBEFORECLICKS",
			1200);
	this->moveAfterTensionDrop = autoKLoad->getConstant("KMOVEAFTERCLICKS",
			4000);
	this->foldTime = autoKLoad->getConstant("KFOLDTIME", 5000);
	autoTimer = new Timer();
	this->winch = winch;
	right = new Encoder(0, 1);
	left = new Encoder(2, 3);
}

AutonomousRoutine::~AutonomousRoutine() {
	// TODO Auto-generated destructor stub
}

void AutonomousRoutine::start() {
	SmartDashboard::PutBoolean("Run", true);
	pin->Set(true); // Release the stinger ASAP
	autoTimer->Start();
	while (autoTimer->Get() < stingerTime / 1000.0) {
		delayMillis(5);
	}
	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	left->Reset();
	while (left->Get() > -moveBeforeTensionDrop) {
		// once we get the recycling containers, bring them back to our side
		drive->TankDrive(-1, -1); // Gotta go fast - Sanic Heghog
		SmartDashboard::PutNumber("Clicks", left->Get());
		delayMillis(5);
	}
	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	left->Reset();
	while (left->Get() > -moveAfterTensionDrop) {
		//get to the auto zone
		drive->TankDrive(-1, -1);
		delayMillis(5);
	}

	//stop once we are in the auto zone
	// Rachel is not the father!
	drive->TankDrive(0.0, 0.0);
	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	while (autoTimer->Get() < foldTime / 1000.0) {
		winch->Set(1.0); // Run the winch
		delayMillis(5);
	}
	winch->Set(0.0); // Stop the winch
	disengageHooks->Set(true);
	pin->Set(false);
	delayMillis(400);
	disengageHooks->Set(false);
	SmartDashboard::PutBoolean("Run", false);
}
