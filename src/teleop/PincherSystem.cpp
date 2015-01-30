#include <teleop/PincherSystem.h>

/*
 *  Rolly grabbers  (pinchers)
	run in and reverse functions
	actuate pistons on the arms independently and together (plan for both)
	4 positions total
	Arms have to be out when the elevator moves
	Are assisting in tote pickup
	Rolly grabbers run until bottom proxy triggered
	Raise RC onto top fingers while driving to human player station
 *
 */

PincherSystem::PincherSystem() {

	rightToteAccel = new VictorSP(6); // TODO ports for all of these
	leftToteAccel = new VictorSP(3);
	rightRollers = new VictorSP(9);
	leftRollers = new VictorSP(0);


	// True = open
	// False = closed
	// NEEDS TO BE CONSISTANT THROUGHOUT
	openPinchers = new DoubleSolenoid(0, 3);

	bottomSensor = new AnalogInput(1);
	bottomSensor = new AnalogInput(0);

	pinchersOpen = false; // Could changed, depends on solenoid. However, this shouldn't change.
	rightOpen = false;
	leftOpen = false;
}

PincherSystem::~PincherSystem() { }



void PincherSystem::OpenPinchers() {
	pinchersOpen = true;
	openPinchers->Set(DoubleSolenoid::Value::kForward);
}

 void PincherSystem::ClosePinchers() {
	 pinchersOpen = false;
	 openPinchers->Set(DoubleSolenoid::Value::kReverse);
}

void PincherSystem::TogglePinchers() {
	pinchersOpen = !pinchersOpen;
	if(pinchersOpen) {
		openPinchers->Set(DoubleSolenoid::Value::kForward);
	}
	else {
		openPinchers->Set(DoubleSolenoid::Value::kReverse);
	}

}


// http://wpilib.screenstepslive.com/s/4485/m/13810/l/241876-analog-inputs
bool PincherSystem::BottomProximityTriggered() {
	if (bottomSensor->GetValue() > 1300){ //TODO change based on how close we want the tote
		return true;
	}
	else {
		return false;
	}
}

bool PincherSystem::TopProximityTriggered() {
	if (topSensor->GetValue() > 1300){ //TODO change based on how close we want the tote
		return true;
	}
	else {
		return false;
	}
}

bool PincherSystem::OpenState(){
	return pinchersOpen;
}

void PincherSystem::RunPinchers() {
	rightRollers->Set(1.0);
	leftRollers->Set(1.0);
	rightToteAccel->Set(1.0);
	leftToteAccel->Set(1.0);
}

void PincherSystem::RunToteAccel() {
	rightToteAccel->Set(1.0);
	leftToteAccel->Set(1.0);
}

void PincherSystem::RunAt(float x){
	rightRollers->Set(x);
	leftRollers->Set(x);
	rightToteAccel->Set(x);
	leftToteAccel->Set(x);
}


void PincherSystem::ReversePinchers() {
	rightRollers->Set(-1.0);
	leftRollers->Set(-1.0);
	rightToteAccel->Set(-1.0);
	leftToteAccel->Set(-1.0);
}

void PincherSystem::ReverseToteAccel() {
	rightToteAccel->Set(-1.0);
	leftToteAccel->Set(-1.0);
}

void PincherSystem::ReversePinchersSlow() {
	rightToteAccel->Set(0.2);
	leftToteAccel->Set(0.2);
}


void PincherSystem::HumanLoad(bool pressed, bool reverse) {
	if (pressed && !reverse)
	{
		RunToteAccel();
		if (BottomProximityTriggered()) {
			rightRollers->Set(0.0);
			leftRollers->Set(0.0);
		}
		else {
			RunPinchers();
		}
		if(pinchersOpen){
			ClosePinchers();
		}
	}
	else
	{
		StopPinchers();
	}

}

void PincherSystem::StopPinchers() {
	rightRollers->Set(0.0);
	leftRollers->Set(0.0);
	rightToteAccel->Set(0.0);
	leftToteAccel->Set(0.0);

}

