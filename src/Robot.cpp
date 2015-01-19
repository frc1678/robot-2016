
//#include "string"
//#include "teleop/CitrusButton.h"
#include "Robot.h"

void Robot::RobotInit() {
//	lw = LiveWindow::GetInstance();
	//SmartDashboard::init();


	driverL = new Joystick(0);
	driverR = new Joystick(1);
	manipulator = new Joystick(2);

	gearUp = new CitrusButton(driverL, 2);
	gearDown = new CitrusButton(driverR, 2);

	shifting = new DoubleSolenoid(1, 2);

	drivetrain = new RobotDrive(2, 7, 1, 8);
	drivetrain->SetSafetyEnabled(false);

	elevator = new ElevatorSystem();



}

void Robot::DisabledInit() {





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
//
	runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);

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


//	SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
//	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());



	if (gearUp->ButtonClicked()) {
		shifting->Set(DoubleSolenoid::Value::kReverse);
	} else if (gearDown->ButtonClicked()) {
		shifting->Set(DoubleSolenoid::Value::kForward);
	} else {

		shifting->Set(DoubleSolenoid::Value::kOff);
	}

	UpdateButtons();

}

void Robot::TestPeriodic()
{
//	lw->Run();
}

void Robot::UpdateButtons()
{
	gearDown->Update();
	gearUp->Update();
}

START_ROBOT_CLASS(Robot);
