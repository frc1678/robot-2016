
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

	//	gearUp = new CitrusButton(driverL, 2);
	//	gearDown = new CitrusButton(driverR, 2);
	// mag3 = new CitrusButton(manipulator, 3);
	// mag4 = new CitrusButton(manipulator, 4);
	openPinchers = new CitrusButton(manipulator, 1);
	closePinchers = new CitrusButton(manipulator, 2);
	runPinchers = new CitrusButton(manipulator, 3);
	//SteeringWheelChoice = new CitrusButton(speedJoystick, 5);


	shifting = new DoubleSolenoid(1, 2);

	//drivetrain = new RobotDrive(2, 7, 1, 8);
	drivetrain->SetSafetyEnabled(false);

	//elevator = new ElevatorSystem();

	pinchers = new PincherSystem();

}

void Robot::DisabledInit() {






	//	if (!elevator->FullyCalibrated()) {
	//		elevator->Reset();
	//	}

	//	compressor->SetClosedLoopControl(false);
	//
	//	sinSol = new Solenoid(7);
	//
	//	gearUp = new CitrusButton(driverL, 2);
	//	gearDown = new CitrusButton(driverR, 2);
	//	mag3 = new CitrusButton(manipulator, 3);
	//	mag4 = new CitrusButton(manipulator, 4);
	//
	//	if (!elevator->FullyCalibrated()) {
	//		elevator->Reset();
	//	}


}

void Robot::DisabledPeriodic() {


	//	if (!elevator->FullyCalibrated()) {
	//		elevator->Calibrate();
	//	}

	//	SmartDashboard::PutNumber("Counter",
	//			static_cast<double>(elevator->elvEncoder->Get()));
	//	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());

	//	compressor->SetClosedLoopControl(false);
	//
	//	if (!elevator->FullyCalibrated()) {
	//		elevator->Calibrate();
	//	}
	//
	//	SmartDashboard::PutNumber("Counter",
	//			static_cast<double>(elevator->elvEncoder->Get()));
	//	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

	//	elevator->StartPIDMag(1);

	//	triggered3 = false;
	//	triggered4 = false;
}



void Robot::TeleopPeriodic() {
	//	compressor->SetClosedLoopControl(true);
//	SmartDashboard::PutNumber("Speed", speedJoystick->GetY());
	// runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
	//swd->drive(SteeringWheelChoice->ButtonPressed() ? 1 : 0);

	if(runPinchers->ButtonPressed()) {
		pinchers->RunPinchers();
	}
	else {
		pinchers->StopPinchers();
	}

	if(openPinchers->ButtonClicked()) {
		pinchers->OpenPinchers();
	}
	else if (closePinchers->ButtonClicked()) {
		pinchers->ClosePinchers();
	}

	//		if(mag3->ButtonClicked()) {
	//			elevator->StartPIDMag(1);
	//			triggered3 = true;
	//		}
	//		else if (mag4->ButtonClicked()) {
	//			elevator->StartPIDMag(3);
	//			triggered4 = true;
	//		}
	//
	//		if (triggered3 && !elevator->AtPosition()) {
	//			elevator->MoveToMagnet(1);
	//		}
	//
	//		if (triggered4 && !elevator->AtPosition()) {
	//			elevator->MoveToMagnet(3);
	//		}

	//	elevator->MoveToMagnet(1);
	//
	//	//		elv1->Set(driverL->GetY());
	//	//		elv2->Set(driverL->GetY());
	//
	//	SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
	//	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());
	//
	//	if (gearUp->ButtonClicked()) {
	//		shifting->Set(DoubleSolenoid::Value::kReverse);
	//	} else if (gearDown->ButtonClicked()) {
	//		shifting->Set(DoubleSolenoid::Value::kForward);
	//	} else {
	//
	//		shifting->Set(DoubleSolenoid::Value::kOff);
	//	}
	//SteeringWheelChoice->Update();
	//	UpdateButtons();


}

void Robot::TestPeriodic()
{
	//	lw->Run();
}

void Robot::UpdateButtons()
{
	gearDown->Update();
	gearUp->Update();
	openPinchers->Update();
	closePinchers->Update();
	runPinchers->Update();
}

START_ROBOT_CLASS(Robot);
