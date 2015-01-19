#include "WPILib.h"
#include "logs/ConstantsLoader.h"
#include "teleop/CitrusButton.h"
#include "teleop/ElevatorSystem.h"
#include "teleop/Drivetrain.h"
#include "string"
#include "teleop/CitrusButton.h"
#include "Robot.h"

#define MIN(a, b) a < b ? a : b
#define ABS(a) a > 0 ? a : -a

void Robot::RobotInit() {
	lw = LiveWindow::GetInstance();
	SmartDashboard::init();

	//kLoad = new ConstantsLoader("auto.txt");

	drivetrain = new RobotDrive(2, 7, 1, 8);
	//	driverL = new Joystick(0);
	//	driverR = new Joystick(1);
	steeringWheel = new Joystick(0);
	speedJoystick = new Joystick(1);

	//	gearUp = new CitrusButton(driverL, 2);
	//	gearDown = new CitrusButton(driverR, 2);
	// mag3 = new CitrusButton(manipulator, 3);
	// mag4 = new CitrusButton(manipulator, 4);
	SteeringWheelChoice=new CitrusButton(speedJoystick, 5);

	//
	//		elv1 = new Talon(4);
	//		elv2 = new Talon(5);
}

void Robot::DisabledInit() {

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
	//	compressor->SetClosedLoopControl(true);
	//	elevator->StartPIDMag(1);
	//
	//	triggered3 = false;
	//	triggered4 = false;
}

void Robot::TeleopPeriodic() {
	//	compressor->SetClosedLoopControl(true);
	SmartDashboard::PutNumber("Speed", speedJoystick->GetY());
	float speed = speedJoystick->GetY();

	float LOutputRel=MIN(1, steeringWheel->GetX()*2+1);
	float ROutputRel=MIN(1, -steeringWheel->GetX()*2+1);
	SmartDashboard::PutNumber("LOutputRel", LOutputRel);
	SmartDashboard::PutNumber("ROutputRel", ROutputRel);
	float LOutputDrive=LOutputRel*speed;
	float ROutputDrive=ROutputRel*speed;
	SmartDashboard::PutNumber("LOutputDrive", LOutputDrive);
	SmartDashboard::PutNumber("ROutputDrive", ROutputDrive);

	float lCoeff = 1 + steeringWheel->GetX();
	float rCoeff = 1 - steeringWheel->GetX();


	//TODO: add a dead zone
	if (-.2 < speed && .2 > speed && (steeringWheel->GetX() < -.15 || steeringWheel->GetX() > .15))
	{

		speed = -.5;
		if (steeringWheel->GetX() < 0)
		{
			rCoeff -= .15;
			lCoeff = -rCoeff;
		}
		else if (steeringWheel->GetX() > 0)
		{
			lCoeff -= .15;
			rCoeff = -lCoeff;
		}
		else
		{
			rCoeff = 0;
			lCoeff = 0;
		}
	}

//	drivetrain->TankDrive(lCoeff * speed, rCoeff * speed);
	drivetrain->TankDrive(LOutputDrive, ROutputDrive);


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

	//	UpdateButtons();

}

void Robot::TestPeriodic()
{
	lw->Run();
}

void Robot::UpdateButtons()
{
	gearDown->Update();
	gearUp->Update();
}

START_ROBOT_CLASS(Robot);
