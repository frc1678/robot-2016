#include <teleop/ElevatorSystem.h>

ElevatorSystem::ElevatorSystem(PositioningSystem* pos, RobotDrive* drive, Solenoid *db, Encoder* right, Encoder* l) {
	elevTimer = new Timer();

	this->drive = drive;
	this->pos = pos;
	this->diskBreak = db;
	this->leftEncoder = l;
	this->rightEncoder = right;

	kLoad = new ConstantsLoader("ElevatorConstants.txt");

	//prefs = new Preferences();
	//prefs->GetInstance();

	//prefs->GetFloat("uKP", )


	n = 0.0;
	sum = 0.0;
	offset = 0.0;

	elvMotor = new VictorSP(5);

	// CONTROL LOOPS FOR THE STATE MACHINE
	ReloadConstants();

	pidLoop = NULL;

	magClicksIndex = 0;

	clicks = 0.0;
//		oldclicks = 0.0;

	elvEncoder = new Encoder(14, 15);
	hallSensor = new DigitalInput(0);

	oldstate = false; // Initializing oldstate because elevator starts on a magnet
}

ElevatorSystem::~ElevatorSystem() {
}

void ElevatorSystem::ReloadConstants() {
	kLoad->reload();

	if(this->driveStraightPID != NULL) {
		delete driveStraightPID;

		delete upPIDOne;
		delete downPIDOne;
		delete upPIDTwo;
		delete downPIDOne;
		delete upPIDThree;
		delete downPIDThree;
		delete upPIDFour;
		delete downPIDFour;
		delete upPIDFive;
		delete downPIDFive;
		delete upPIDSix;
		delete downPIDSix;
		delete upPIDSeven;
		delete downPIDSeven;

		delete upPID;
		delete downPID;
	}

	this->driveStraightPID = new ControlLoop(kLoad->getConstant("DRIVESTRAIGHTP"), kLoad->getConstant("DRIVESTRAIGHTI"), kLoad->getConstant("DRIVESTRAIGHTD"));

	{
		uKP1 = kLoad->getConstant("UKP1");
		uKI1 = kLoad->getConstant("UKI1");
		uKD1 = kLoad->getConstant("UKD1");

		uKP2 = kLoad->getConstant("UKP2");
		uKI2 = kLoad->getConstant("UKI2");
		uKD2 = kLoad->getConstant("UKD2");

		uKP3 = kLoad->getConstant("UKP3");
		uKI3 = kLoad->getConstant("UKI3");
		uKD3 = kLoad->getConstant("UKD3");

		uKP4 = kLoad->getConstant("UKP4");
		uKI4 = kLoad->getConstant("UKI4");
		uKD4 = kLoad->getConstant("UKD4");

		uKP5 = kLoad->getConstant("UKP5");
		uKI5 = kLoad->getConstant("UKI5");
		uKD5 = kLoad->getConstant("UKD5");

		uKP6 = kLoad->getConstant("UKP6");
		uKI6 = kLoad->getConstant("UKI6");
		uKD6 = kLoad->getConstant("UKD6");

		uKP7 = kLoad->getConstant("UKP7");
		uKI7 = kLoad->getConstant("UKI7");
		uKD7 = kLoad->getConstant("UKD7");



		dKP1 = kLoad->getConstant("DKP1");
		dKI1 = kLoad->getConstant("DKI1");
		dKD1 = kLoad->getConstant("DKD1");

		dKP2 = kLoad->getConstant("DKP2");
		dKI2 = kLoad->getConstant("DKI2");
		dKD2 = kLoad->getConstant("DKD2");

		dKP3 = kLoad->getConstant("DKP3");
		dKI3 = kLoad->getConstant("DKI3");
		dKD3 = kLoad->getConstant("DKD3");

		dKP4 = kLoad->getConstant("DKP4");
		dKI4 = kLoad->getConstant("DKI4");
		dKD4 = kLoad->getConstant("DKD4");

		dKP5 = kLoad->getConstant("DKP5");
		dKI5 = kLoad->getConstant("DKI5");
		dKD5 = kLoad->getConstant("DKD5");

		dKP6 = kLoad->getConstant("DKP6");
		dKI6 = kLoad->getConstant("DKI6");
		dKD6 = kLoad->getConstant("DKD6");

		dKP7 = kLoad->getConstant("DKP7");
		dKI7 = kLoad->getConstant("DKI7");
		dKD7 = kLoad->getConstant("DKD7");

		SmartDashboard::PutNumber("dKD5", dKD5);
	}

	dropTime = kLoad->getConstant("DROPSTACKTIME", 1000);
	driveClicks = kLoad->getConstant("DRIVESTACKCLICKS", 3000);
	driveSpeed = kLoad->getConstant("DRIVESTACKSPEED", .3);


	upPIDOne = new ControlLoop(uKP1, uKI1, uKD1);
	downPIDOne = new ControlLoop(dKP1, dKI1, dKD1);

	upPIDTwo = new ControlLoop(uKP2, uKI2, uKD2);
	downPIDTwo = new ControlLoop(dKP2, dKI2, dKD2);

	upPIDThree = new ControlLoop(uKP3, uKD3, uKD3);
	downPIDThree = new ControlLoop(dKP3, dKD3, dKD3);

	upPIDFour = new ControlLoop(uKP4, uKI4, uKD4);
	downPIDFour = new ControlLoop(dKP4, dKI4, dKD4);

	upPIDFive = new ControlLoop(uKP5, uKI5, uKD5);
	downPIDFive = new ControlLoop(dKP5, dKI5, dKD5);

	upPIDSix = new ControlLoop(uKP6, uKI6, uKD6);
	downPIDSix = new ControlLoop(dKP6, dKI6, dKD6);

	upPIDSeven = new ControlLoop(uKP7, uKI7, uKD7);
	downPIDSeven = new ControlLoop(dKP7, dKI7, dKD7);

	upPID = new ControlLoop(uKP, uKI, uKD);
	downPID = new ControlLoop(dKP, dKI, dKD);
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
	magClicksIndex = 0;
}

bool ElevatorSystem::FullyCalibrated() {
	return n == 2 * (sizeof(ABS_MAG_CLICKS) / sizeof(float)) - 1;
}

void ElevatorSystem::StartPIDMag(int mag) {
	StartPID(ABS_MAG_CLICKS[mag]);
}

void ElevatorSystem::StartPIDPosition(int pos) {
	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_ELEVATOR_POSITIONS[pos];

	if (pos == 0) {
		if (error >= 0) {
			pidLoop = upPIDOne;
		} else {
			pidLoop = downPIDOne;
		}
	} else if (pos == 1) {
		if (error >= 0) {
			pidLoop = upPIDTwo;
		} else {
			pidLoop = downPIDTwo;
		}
	} else if (pos == 2) {
		if (error >= 0) {
			pidLoop = upPIDThree;
		} else {
			pidLoop = downPIDThree;
		}
	} else if (pos == 3) {
		if (error >= 0) {
			pidLoop = upPIDFour;
		} else {
			pidLoop = downPIDFour;
		}
	} else if (pos == 4) {
		if (error >= 0) {
			pidLoop = upPIDFive;
		} else {
			pidLoop = downPIDFive;
		}
	} else if (pos == 5) {
		if (error >= 0) {
			pidLoop = upPIDSix;
		} else {
			pidLoop = downPIDSix;
		}
	} else if (pos == 6) {
		if (error >= 0) {
			pidLoop = upPIDSeven;
		} else {
			pidLoop = downPIDSeven;
		}
	}

	done = false;

	pidLoop->StartLoop();

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

	if(pidLoop != NULL) {
		pidLoop->Stop();
		pidLoop->ResetLoop();
		pidLoop = NULL;
		done = true;
	}
}

void ElevatorSystem::MoveToAbsEncoderPosition(int encoderPos) {
	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - encoderPos;

	SmartDashboard::PutNumber("Abs_pos", absPos);
	SmartDashboard::PutNumber("Elevator error", error);

	float pidOut = 0;
	// Terminating Condition
	bool done = abs(error) < termination || pidLoop == NULL;
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

	elvMotor->Set(pidOut);

	done = abs(error) < termination || (abs(error) < 4*termination && abs(pidOut) < 0.09);
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

void ElevatorSystem::MoveTo_One_PickupRC() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[0]);
}

void ElevatorSystem::MoveTo_Two_BackupFromStack() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[1]);
}

void ElevatorSystem::MoveTo_Three_PrepStackPickup() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[2]);
}

void ElevatorSystem::MoveTo_Four_HoldStack() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[3]);
}

void ElevatorSystem::MoveTo_Five_PrepHPOne() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[4]);
}

void ElevatorSystem::MoveTo_Six_HPWaitRC() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[5]);
}

void ElevatorSystem::MoveTo_Seven_HPWaitTote() {
	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[6]);
}

void ElevatorSystem::MoveElevator(float motorInput) {
	elvMotor->Set(motorInput);
}

void ElevatorSystem::DropStack() {
	// TODO Drop the stack somehow
	// Wait for a little bit
	elevTimer->Reset();
	elevTimer->Start();
	while (elevTimer->Get() < dropTime / 1000.0) {
		delayMillis(5);
	}
	// Back off with a PID loop to drive straight
	elevTimer->Reset();
	elevTimer->Start();
	double angleAtStart = pos->getAngle();
	while (min(rightEncoder->Get(), leftEncoder->Get()) < driveClicks)
	{
		double adjustment = driveStraightPID->CalibrateLoop(pos->getAngle() - angleAtStart);
		double adjustedDriveL = 1, adjustedDriveR = 1;
		adjustedDriveL = adjustedDriveL * (1 - adjustment);
		adjustedDriveR = adjustedDriveR * (1 + adjustment);
		double normalizer = 1.0 / (adjustedDriveL > adjustedDriveR ? adjustedDriveL : adjustedDriveR);
		adjustedDriveL *= normalizer * driveSpeed;
		adjustedDriveR *= normalizer * driveSpeed;
		drive->TankDrive(driveSpeed, driveSpeed);
	}
}

void ElevatorSystem::SetDiskBreak(bool on) {
	diskBreak->Set(on);
}
