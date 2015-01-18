#include <string>
#include "WPILib.h"
#include "Drivetrain.h"
#include "ElevatorSystem.h"
#include "ControlLoop.h"
#include "CitrusButton.h"
#include "ConstantsLoader.h"
#include "Robot.h"

void Robot::RobotInit()
{
<<<<<<< HEAD
private:
	LiveWindow *lw;

	// Joysticks that Takumi drives with
	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator;

	// For the driving of the robot
	RobotDrive *drivetrain;

	// The elevator
	ElevatorSystem *elevator;

	Compressor *compressor;

	Gyro *gyro;

	Solenoid *sinSol;
	DoubleSolenoid *shifting;
=======
	lw = LiveWindow::GetInstance();
	SmartDashboard::init();
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1

	kLoad = new ConstantsLoader();

	driverL = new Joystick(0);
	driverR = new Joystick(1);
	manipulator = new Joystick(2);

	left = new VictorSP(1);
	right = new VictorSP(0);

<<<<<<< HEAD
=======
	drivetrain = new RobotDrive(left, right);
	drivetrain->SetSafetyEnabled(false);

	elevator = new ElevatorSystem();
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1

	compressor = new Compressor(0);

	sinSol = new Solenoid(7);
	dbSol = new DoubleSolenoid(0, 1);

<<<<<<< HEAD
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		SmartDashboard::init();
		
		kLoad = new ConstantsLoader("auto.txt");
		fsave = new FileSave("log.csv");
=======
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1


	gearUp = new CitrusButton(driverL, 2);
	gearDown = new CitrusButton(driverR, 2);
	mag3 = new CitrusButton(manipulator, 3);
	mag4 = new CitrusButton(manipulator, 4);

	//
	//		elv1 = new Talon(4);
	//		elv2 = new Talon(5);
}

void Robot::DisabledInit() {

	compressor->SetClosedLoopControl(false);

<<<<<<< HEAD
		sinSol = new Solenoid(7);
		shifting = new DoubleSolenoid(0, 1);



		gearUp = new CitrusButton(driverL, 2);
		gearDown = new CitrusButton(driverR, 2);
		mag3 = new CitrusButton(manipulator, 3);
		mag4 = new CitrusButton(manipulator, 4);
		
		fsave->start();
		//fsave->logRobotInit();
		fsave->flush();


=======
	if(!elevator->FullyCalibrated()){
		elevator->Reset();
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1
	}

}

void Robot::DisabledPeriodic() {

	compressor->SetClosedLoopControl(false);

	if(!elevator->FullyCalibrated()){
		elevator->Calibrate();
	}

	SmartDashboard::PutNumber("Counter", static_cast<double>(elevator->elvEncoder->Get()));
	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());
}

void Robot::AutonomousInit()
{

}

void Robot::AutonomousPeriodic()
{

}

void Robot::TeleopInit()
{
	compressor->SetClosedLoopControl(true);
	elevator->StartPIDMag(1);

	triggered3 = false;
	triggered4 = false;
}



void Robot::TeleopPeriodic()
{
	compressor->SetClosedLoopControl(true);


<<<<<<< HEAD
	void TeleopPeriodic()
	{
		compressor->SetClosedLoopControl(true);
		
		fsave->start();
		//fsave->logRobot(test, test, leftEncoder, rightEncoder, ds);
		fsave->flush();
=======
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1


	//		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);

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

	elevator->MoveToMagnet(1);

<<<<<<< HEAD
=======
	//		elv1->Set(driverL->GetY());
	//		elv2->Set(driverL->GetY());
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1

	SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
	SmartDashboard::PutNumber("Avg", elevator->AvgOffset());



<<<<<<< HEAD
		if(gearUp->ButtonClicked()){
			shifting->Set(DoubleSolenoid::Value::kReverse);
		}
		else if(gearDown->ButtonClicked()){
			shifting->Set(DoubleSolenoid::Value::kForward);
		}
		else {

			shifting->Set(DoubleSolenoid::Value::kOff);
		}
=======
	if(gearUp->ButtonClicked()){
		dbSol->Set(DoubleSolenoid::Value::kReverse);
	}
	else if(gearDown->ButtonClicked()){
		dbSol->Set(DoubleSolenoid::Value::kForward);
	}
	else {

		dbSol->Set(DoubleSolenoid::Value::kOff);
	}
>>>>>>> 923cdd1b233348349f32bc899e233045f2e9abc1



	UpdateButtons();

}

void Robot::TestPeriodic()
{
	lw->Run();
}


void Robot::UpdateButtons() {
	gearDown->Update();
	gearUp->Update();

}

START_ROBOT_CLASS(Robot);
