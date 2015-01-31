#include <teleop/ElevatorSystem.h>
#define NULL 0

ElevatorSystem::ElevatorSystem(PositioningSystem* pos, RobotDrive* drive, Encoder* right, Encoder* left) {
	elevTimer = new Timer();

	this->drive = drive;
	this->pos = pos;
	this->leftEncoder = left;
	this->rightEncoder = right;


	//prefs = new Preferences();
	//prefs->GetInstance();

	//prefs->GetFloat("uKP", )
	ConstantsLoader kLoad("ElevatorConstants.txt");
	this->driveStraightPID = new ControlLoop(kLoad.getConstant("DRIVESTRAIGHTP"), kLoad.getConstant("DRIVESTRAIGHTI"), kLoad.getConstant("DRIVESTRAIGHTD"));
	{
		uKP1 = kLoad.getConstant("UKP1");
		uKI1 = kLoad.getConstant("UKI1");
		uKD1 = kLoad.getConstant("UKD1");
		uKP3 = kLoad.getConstant("UKP3");
		uKI3 = kLoad.getConstant("UKI3");
		uKD3 = kLoad.getConstant("UKD3");
		uKP4 = kLoad.getConstant("UKP4");
		uKI4 = kLoad.getConstant("UKI4");
		uKD4 = kLoad.getConstant("UKD4");
		uKP7 = kLoad.getConstant("UKP7");
		uKI7 = kLoad.getConstant("UKI7");
		uKD7 = kLoad.getConstant("UKD7");
		uKP8 = kLoad.getConstant("UKP8");
		uKI8 = kLoad.getConstant("UKI8");
		uKD8 = kLoad.getConstant("UKD8");

		dKP1 = kLoad.getConstant("DKP1");
		dKI1 = kLoad.getConstant("DKI1");
		dKD1 = kLoad.getConstant("DKD1");
		dKP3 = kLoad.getConstant("DKP3");
		dKI3 = kLoad.getConstant("DKI3");
		dKD3 = kLoad.getConstant("DKD3");
		dKP4 = kLoad.getConstant("DKP4");
		dKI4 = kLoad.getConstant("DKI4");
		dKD4 = kLoad.getConstant("DKD4");
		dKP7 = kLoad.getConstant("DKP7");
		dKI7 = kLoad.getConstant("DKI7");
		dKD7 = kLoad.getConstant("DKD7");
		dKP8 = kLoad.getConstant("DKP8");
		dKI8 = kLoad.getConstant("DKI8");
		dKD8 = kLoad.getConstant("DKD8");
	}

	dropTime = kLoad.getConstant("DROPSTACKTIME", 1000);
	driveClicks = kLoad.getConstant("DRIVESTACKCLICKS", 3000);
	driveSpeed = kLoad.getConstant("DRIVESTACKSPEED", .3);

	n = 0.0;
	sum = 0.0;
	offset = 0.0;

	elvMotor = new VictorSP(5);

	// CONTROL LOOPS FOR THE STATE MACHINE
	upPIDOne = new ControlLoop(uKP1, uKI1, uKD1);
	downPIDOne = new ControlLoop(dKP1, dKI1, dKD1);
	upPIDThree = new ControlLoop(uKP3, uKD3, uKD3);
	downPIDThree = new ControlLoop(dKP3, dKD3, dKD3);
	upPIDFour = new ControlLoop(uKP4, uKI4, uKD4);
	downPIDFour = new ControlLoop(dKP4, dKI4, dKD4);
	upPIDSeven = new ControlLoop(uKP7, uKI7, uKD7);
	downPIDSeven = new ControlLoop(dKP7, dKI7, dKD7);
	upPIDEight = new ControlLoop(uKP8, uKI8, uKD8);
	downPIDEight = new ControlLoop(dKP8, dKI8, dKD8);

	upPID = new ControlLoop(uKP, uKI, uKD);
	downPID = new ControlLoop(dKP, dKI, dKD);
	pidLoop = NULL;

	magClicksIndex = 1;

	clicks = 0.0;
//		oldclicks = 0.0;

	elvEncoder = new Encoder(3, 2);
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
			pidLoop = upPIDThree;
		} else {
			pidLoop = downPIDThree;
		}
	} else if (pos == 2) {
		if (error >= 0) {
			pidLoop = upPIDFour;
		} else {
			pidLoop = downPIDFour;
		}
	} else if (pos == 3) {
		if (error >= 0) {
			pidLoop = upPIDSeven;
		} else {
			pidLoop = downPIDSeven;
		}
	} else if (pos == 4) {
		if (error >= 0) {
			pidLoop = upPIDEight;
		} else {
			pidLoop = downPIDEight;
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

	elvMotor->Set(pidOut);

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

	elvMotor->Set(pidOut);

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

void ElevatorSystem::MoveToGround() {
	if (done) {
		return;
	}

	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_ELEVATOR_POSITIONS[0];

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

	elvMotor->Set(pidOut);

	if (done) {
		StopPID();
	}

}

void ElevatorSystem::MoveToScoringPosition() {
	if (done) {
		return;
	}

//	MoveToAbsEncoderPosition(ABS_ELEVATOR_POSITIONS[1]);
	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_ELEVATOR_POSITIONS[1];

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

	elvMotor->Set(pidOut);

	if (done) {
		StopPID();
	}

}

void ElevatorSystem::MoveToHPLoadOne() {
	if (done) {
		return;
	}

	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_ELEVATOR_POSITIONS[2];

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

	elvMotor->Set(pidOut);

	if (done) {
		StopPID();
	}

}

void ElevatorSystem::MoveToHPLoadTwo() {
	if (done) {
		return;
	}

	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_ELEVATOR_POSITIONS[3];

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

	elvMotor->Set(pidOut);

	if (done) {
		StopPID();
	}

}

void ElevatorSystem::MoveToStationaryPosition() {
	if (done) {
		return;
	}

	int absPos = elvEncoder->Get() - AvgOffset();
	int error = absPos - ABS_ELEVATOR_POSITIONS[4];

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

	elvMotor->Set(pidOut);

	if (done) {
		StopPID();
	}

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
