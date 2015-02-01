#include "Robot.h"

#define MIN(a, b) a < b ? a : b
#define ABS(a) a > 0 ? a : -a

void Robot::RobotInit() {
	//	lw = LiveWindow::GetInstance();
	//SmartDashboard::init();

	//kLoad = new ConstantsLoader("auto.txt");

	drivetrain = new RobotDrive(2, 7, 1, 8);
	driverL = new Joystick(0);
	driverR = new Joystick(1);
	manipulator = new Joystick(2);
	//steeringWheel = new Joystick(0);
	//speedJoystick = new Joystick(1);
	//swd = new SteeringWheelDrive(drivetrain, steeringWheel, speedJoystick, new ConstantsLoader("joystick.txt"));

	shifting = new DoubleSolenoid(1, 2);
	diskBreak = new Solenoid(6); // TODO: change this to the right number

	rightEncoder = new Encoder(10, 11);
	leftEncoder = new Encoder(12, 13);

	drivetrain->SetSafetyEnabled(false);

	pinchers = new PincherSystem();

	this->autoCode = new AutonomousRoutine(new Solenoid(4), new Solenoid(5),
			drivetrain, new ConstantsLoader("joystick.txt"), new Victor(4),
			rightEncoder, leftEncoder);

	gearUp = new CitrusButton(driverL, 2);
	gearDown = new CitrusButton(driverR, 2);
	straightButton = new CitrusButton(driverR, 3);
	openPinchers = new CitrusButton(manipulator, 1);
	closePinchers = new CitrusButton(manipulator, 2);
	runPinchers = new CitrusButton(driverR, 1);
	reversePinchers = new CitrusButton(driverL, 1);
	//SteeringWheelChoice = new CitrusButton(speedJoystick, 5);

	// Testing buttons for going to individual elevator positions: Probably more convenient for tunning.
	currentElvTarget = -1;
	e1 = new CitrusButton(driverR, 1);
	e2 = new CitrusButton(driverR, 2);
	e3 = new CitrusButton(driverR, 3);
	e4 = new CitrusButton(driverR, 4);
	e5 = new CitrusButton(driverR, 5);
	e6 = new CitrusButton(driverR, 6);
	e7 = new CitrusButton(driverR, 7);

	elevator = new ElevatorSystem(pos, drivetrain, diskBreak, rightEncoder, leftEncoder);

	elevatorStateMachine = new StateMachine(elevator, pinchers);

	ElevLog = new CSVLogger("ElevatorLog", "Encoder,MotorOutput,PIDConstant");

	this->pos = new PositioningSystem();

}

void Robot::DisabledInit() {
//
	if (!elevator->FullyCalibrated()) {
		elevator->Reset();
	}

}

void Robot::DisabledPeriodic() {
//
	if (!elevator->FullyCalibrated()) {
		elevator->Calibrate();
	}

	SmartDashboard::PutNumber("Counter",
			static_cast<double>(elevator->elvEncoder->Get()));
	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());

}

void Robot::AutonomousInit() {
	this->autoCode->start();
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
	// TODO: Constants should get reloaded from the file on TeleopInit, so that we can use new constants without restarting robot code.
	// However, this seems to break elevator system... fix this!
//	elevator->ReloadConstants();

//	elevator->StartPIDPosition(4);
}

void Robot::TeleopPeriodic() {
	//pos->update();
	// The dingus has left the building!
//	runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain, straightButton->ButtonPressed());

	// The eagle has left the nest
	//drivetrain->TankDrive(driverL->GetY(), driverR->GetY());
	//swd->drive(SteeringWheelChoice->ButtonPressed() ? 1 : 0);

//	if (runPinchers->ButtonPressed()) {
//		pinchers->RunPinchers();
//	} else {
//		pinchers->StopPinchers();
//	}

	if (openPinchers->ButtonClicked()) {
		pinchers->OpenPinchers();
	} else if (closePinchers->ButtonClicked()) {
		pinchers->ClosePinchers();
	}



	elevatorStateMachine->PeriodicUpdate(e1->ButtonClicked()); // TODO: Change this to use not a testing button as input to the state machine.


	//	SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
	//	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());
	//

	if (gearUp->ButtonClicked()) {
		shifting->Set(DoubleSolenoid::Value::kReverse);
	} else if (gearDown->ButtonClicked()) {
		shifting->Set(DoubleSolenoid::Value::kForward);
	} else {

		shifting->Set(DoubleSolenoid::Value::kOff);
	}

	SmartDashboard::PutNumber("LeftEncoder", leftEncoder->Get());
	SmartDashboard::PutNumber("RightEncoder", rightEncoder->Get());
	SmartDashboard::PutNumber("ElvEncoder", elevator->elvEncoder->Get());

//	SmartDashboard::PutNumber("SensorValue", pinchers->bottomSensor->GetValue());
//
//
//	if (runPinchers->ButtonPressed()){
//		if(pinchers->BottomProximityTriggered()){
//			SmartDashboard::PutBoolean("PinchersThingy", true);
//		}
//		else
//		{
//			SmartDashboard::PutBoolean("PinchersThingy", false);
//		}
//	}
//
//
//	SmartDashboard::PutBoolean("Trigger", runPinchers->ButtonPressed());
//

//	elevator->MoveElevator(driverR->GetY());
//	elevator->MoveTo_Five_PrepHPOne();
//
	if (elevator->elvEncoder != NULL && elevator->pidLoop != NULL) {
		ElevLog->StartNewCycle();
		ElevLog->LogValue(std::to_string(elevator->elvEncoder->Get()));
		ElevLog->LogValue(std::to_string(elevator->pidLoop->kp));
	}

	UpdateButtons();
}

void Robot::TestPeriodic() {

}

void Robot::UpdateButtons() {
	gearDown->Update();
	gearUp->Update();
	openPinchers->Update();
	closePinchers->Update();
	runPinchers->Update();
	reversePinchers->Update();
	straightButton->Update();

	e1->Update();
	e2->Update();
	e3->Update();
	e4->Update();
	e5->Update();
	e6->Update();
	e7->Update();
}

// Call this in TeleopPeriodic() to do per-button testing of the elevator.
void Robot::TestElevatorWithButtons() {
	if (e1->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 0;
		elevator->StartPIDPosition(currentElvTarget);
	}
	if (e2->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 1;
		elevator->StartPIDPosition(currentElvTarget);
	}
	if (e3->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 2;
		elevator->StartPIDPosition(currentElvTarget);
	}
	if (e4->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 3;
		elevator->StartPIDPosition(currentElvTarget);
	}
	if (e5->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 4;
		elevator->StartPIDPosition(currentElvTarget);
	}
	if (e6->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 5;
		elevator->StartPIDPosition(currentElvTarget);
	}
	if (e7->ButtonClicked()) {
		elevator->StopPID();
		currentElvTarget = 6;
		elevator->StartPIDPosition(currentElvTarget);
	}

	if (currentElvTarget == 0) {
		elevator->MoveTo_One_PickupRC();
	}
	if (currentElvTarget == 1) {
		elevator->MoveTo_Two_BackupFromStack();
	}
	if (currentElvTarget == 2) {
		elevator->MoveTo_Three_PrepStackPickup();
	}
	if (currentElvTarget == 3) {
		elevator->MoveTo_Four_HoldStack();
	}
	if (currentElvTarget == 4) {
		elevator->MoveTo_Five_PrepHPOne();
	}
	if (currentElvTarget == 5) {
		elevator->MoveTo_Six_HPWaitRC();
	}
	if (currentElvTarget == 6) {
		elevator->MoveTo_Seven_HPWaitTote();
	}
}

START_ROBOT_CLASS(Robot);
