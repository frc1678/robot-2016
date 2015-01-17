#include "ElevatorSystem.h"

ElevatorSystem::ElevatorSystem() {

	//prefs = new Preferences();
	//prefs->GetInstance();

	//prefs->GetFloat("uKP", )
	n = 0.0;
	sum = 0.0;
	offset = 0.0;

	left = new Talon(5);
	right = new Talon(4);

	upPID = new ControlLoop(uKP, uKI, uKD);
	downPID = new ControlLoop(dKP, dKI, dKD);
	pidLoop = NULL;

	magClicksIndex = 1;

	clicks = 0.0;
//		oldclicks = 0.0;

	elvEncoder = new Encoder(3,2);
	hallSensor = new DigitalInput(8);

	oldstate = false; // Initializing oldstate because elevator starts on a magnet
}

ElevatorSystem::~ElevatorSystem() {
}

void ElevatorSystem::Calibrate() {

	if (FullyCalibrated()) {
		return;
	}

	bool state = HallTriggered();
	clicks = elvEncoder->Get();

	SmartDashboard::PutBoolean("State", state);

	bool isAtTop = magClicksIndex
			== (sizeof(ABS_MAG_CLICKS) / sizeof(float)) - 1;
	if (state && !oldstate) { // Rising edge, the top of the magnet

		sum += FindOffset(ABS_MAG_CLICKS[magClicksIndex], clicks);
		n++;

		if (!goingDown && !isAtTop) {
			magClicksIndex++;
		} else if (goingDown) {
			magClicksIndex--;
		}

		if (isAtTop) {
			goingDown = true;
		}

	} else if (!state && oldstate) { // Falling edge, the bottom of the magnet

		sum += FindOffset(ABS_MAG_CLICKS[magClicksIndex], clicks);
		n++;

		if (!goingDown && !isAtTop) {
			magClicksIndex++;
		} else if (goingDown) {
			magClicksIndex--;
		}

		if (isAtTop) {
			goingDown = true;
		}
	}
	oldstate = state;

}

float ElevatorSystem::AvgOffset() {
	if (n == 0) {
		return 0;
	}

	return sum / n;
}

float ElevatorSystem::FindOffset(float location, float current) {
	float offset = current - location;
	return offset;
}

bool ElevatorSystem::HallTriggered() {
	return !hallSensor->Get();
}

void ElevatorSystem::Reset() {
	elvEncoder->Reset();
	sum = 0.0;
	n = 0.0;
	clicks = 0.0;
//		oldclicks = 0.0;
	offset = 0.0;
//		state = false;
	oldstate = false;
	magClicksIndex = 1;
}

bool ElevatorSystem::FullyCalibrated() {
	return n == 2 * (sizeof(ABS_MAG_CLICKS) / sizeof(float)) - 1;
}

void ElevatorSystem::StartPIDMag(int mag) {
	StartPID(ABS_MAG_CLICKS[mag]);
}

void ElevatorSystem::StartPID(int encoderPos) {

	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - encoderPos;
	if (error >= 0) {
		pidLoop = upPID;
	} else {
		pidLoop = downPID;
	}

	done = false;

	pidLoop->StartLoop();
}

void ElevatorSystem::StopPID() {

	pidLoop->Stop();
	pidLoop->ResetLoop();
	pidLoop = NULL;
}

void ElevatorSystem::MoveToAbsEncoderPosition(int encoderPos) {
	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - encoderPos;

	SmartDashboard::PutNumber("Abs_pos", absPos);
	SmartDashboard::PutNumber("Elevator error", error);

	float pidOut = 0;
	// Terminating Condition
	bool done = abs(error) < termination;
	if (!done) {
		pidOut = pidLoop->CalibrateLoop(error);
	}

	if (pidOut > 1) {
		pidOut = 1;
	}
	if (pidOut < -1) {
		pidOut = -1;
	}

	left->Set(pidOut);
	right->Set(pidOut);

	if (done) {
		StopPID();
	}
}

void ElevatorSystem::MoveToMagnet(int magnet) {
	if (done) {
		return;
	}

	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_MAG_CLICKS[magnet];

	SmartDashboard::PutNumber("Abs_pos", absPos);
	SmartDashboard::PutNumber("Elevator error", error);

	float pidOut = 0;
	done = abs(error) < termination;

	if (!done) {

		pidOut = pidLoop->CalibrateLoop(error);
	}

	Logging(error, pidOut);


	if (pidOut > 1) {
		pidOut = 1;
	}
	if (pidOut < -1) {
		pidOut = -1;
	}

	left->Set(pidOut);
	right->Set(pidOut);


	if (done) {
		StopPID();
	}

}

bool ElevatorSystem::AtPosition() {
	return done;
}

void ElevatorSystem::Logging(float clicks, float motorOutput) {
	encoderErrorFile.open("/home/admin/logs/elevator_error.csv", ios::app);
	encoderErrorFile << clicks << endl;
	encoderErrorFile.close();

	motorOutputErrorFile.open("/home/admin/logs/elevator_output.csv", ios::app);
	motorOutputErrorFile << motorOutput << endl;
	motorOutputErrorFile.close();
}

