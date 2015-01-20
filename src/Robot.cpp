
//#include "string"
//#include "teleop/CitrusButton.h"
#include "Robot.h"

#define MIN(a, b) a < b ? a : b
#define ABS(a) a > 0 ? a : -a

void Robot::RobotInit() {
//	lw = LiveWindow::GetInstance();
	//SmartDashboard::init();

	//kLoad = new ConstantsLoader("auto.txt");

	drivetrain = new RobotDrive(2, 7, 1, 8);
	//	driverL = new Joystick(0);
	//	driverR = new Joystick(1);
	steeringWheel = new Joystick(0);
	speedJoystick = new Joystick(1);
	swd = new SteeringWheelDrive(drivetrain, steeringWheel, speedJoystick, new ConstantsLoader("joystick.txt"));

	//	gearUp = new CitrusButton(driverL, 2);
	//	gearDown = new CitrusButton(driverR, 2);
	// mag3 = new CitrusButton(manipulator, 3);
	// mag4 = new CitrusButton(manipulator, 4);
	SteeringWheelChoice = new CitrusButton(speedJoystick, 5);


	shifting = new DoubleSolenoid(1, 2);

	drivetrain = new RobotDrive(2, 7, 1, 8);
	drivetrain->SetSafetyEnabled(false);

	elevator = new ElevatorSystem();



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
	SmartDashboard::PutNumber("Speed", speedJoystick->GetY());
	// runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
	swd->drive(SteeringWheelChoice->ButtonPressed() ? 1 : 0);

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
	SteeringWheelChoice->Update();
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
}

void Robot::logDrive(float leftEncoderVal, float rightEncoderVal, float REncoderRate, float LEncoderRate, double joy1, double joy2, float LeftMotorOutput, float RightMotorOutput)
{
//	driveLogger->TextLog(std::to_string(leftEncoderVal), "LEFT_ENCODER_VAL", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(rightEncoderVal), "RIGHT_ENCODER_VAL", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(LEncoderRate), "LEFT_ENCODER_RATE", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(REncoderRate), "RIGHT_ENCODER_RATE", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(joy1), "JOYSTICK 1", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(joy2), "JOYSTICK 2", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(LeftMotorOutput), "LEFT_MOTOR_OUTPUT", CODE_STAMP);
//	driveLogger->TextLog(std::to_string(RightMotorOutput), "RIGHT_MOTOR_OUTPUT", CODE_STAMP);
}

void Robot::CSVlogDrive(float LMotorOutput, float RMotorOutput, float LEncoderRate, float REncoderRate)
{
	CSVdriveLogger->StartNewCycle();
	CSVdriveLogger->LogValue(to_string(LMotorOutput));
	CSVdriveLogger->LogValue(to_string(RMotorOutput));
	CSVdriveLogger->LogValue(to_string(LEncoderRate));
	CSVdriveLogger->LogValue(to_string(REncoderRate));
}

START_ROBOT_CLASS(Robot);
