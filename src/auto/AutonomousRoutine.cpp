/*
 * AutonomousRoutine.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <auto/AutonomousRoutine.h>

AutonomousRoutine::AutonomousRoutine(Solenoid* stingerPin,
		Solenoid* disengage, RobotDrive* drive,
		ConstantsLoader* autoKLoad, Victor* winch, Encoder *right, Encoder *left) {
	this->drive = drive;
	this->releasePin = stingerPin;
	this->disengageHooks = disengage;
	this->stingerTime = autoKLoad->getConstant("KSTINGERTIME", 300);
	this->moveBeforeTensionDrop = autoKLoad->getConstant("KMOVEBEFORECLICKS",
			1200);
	this->moveAfterTensionDrop = autoKLoad->getConstant("KMOVEAFTERCLICKS",
			4000);
	this->foldTime = autoKLoad->getConstant("KFOLDTIME", 5000);
	autoTimer = new Timer();
	this->winchMotor = winch;

	rightEncoder = right;
	leftEncoder = left;
}

AutonomousRoutine::~AutonomousRoutine() {
	// TODO Auto-generated destructor stub
}

void AutonomousRoutine::start() {
	SmartDashboard::PutBoolean("Run", true);
	releasePin->Set(true); // Release the stinger ASAP
	autoTimer->Start();

	while (autoTimer->Get() < stingerTime / 1000.0) {
		delayMillis(5);
	}

	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	leftEncoder->Reset();

	while (leftEncoder->Get() > -moveBeforeTensionDrop) {
		// once we get the recycling containers, bring them back to our side
		drive->TankDrive(-1, -1); // Gotta go fast - Sanic Heghog
		SmartDashboard::PutNumber("Clicks", leftEncoder->Get());
		delayMillis(5);
	}

	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();
	leftEncoder->Reset();

	while (leftEncoder->Get() > -moveAfterTensionDrop) {
		//get to the auto zone
		drive->TankDrive(-1, -1);
		delayMillis(5);
	}

	//stop once we are in the auto zone
	// Rachel is not the father!
	// What
	drive->TankDrive(0.0, 0.0);

	// Reset the timer
	autoTimer->Reset();
	autoTimer->Start();

	SmartDashboard::PutBoolean("WinchOn", true);
	SmartDashboard::PutNumber("Winch Time", foldTime);
	while (autoTimer->Get() < foldTime / 1000.0) {
		winchMotor->Set(1.0); // Run the winch
		delayMillis(5);
	}
	SmartDashboard::PutBoolean("WinchOn", false);

	winchMotor->Set(0.0); // Stop the winch
	disengageHooks->Set(true);
	releasePin->Set(false);

	delayMillis(400);
	disengageHooks->Set(false);

	SmartDashboard::PutBoolean("Run", false);
	SmartDashboard::PutNumber("WinchNum", this->winchMotor->GetChannel());

}
