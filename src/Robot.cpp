#include <string>
#include "WPILib.h"
#include "Drivetrain.h"
#include "ElevatorSystem.h"
#include "ControlLoop.h"
#include "CitrusButton.h"
#include "FileSave.h"
#include "ConstantsLoader.h"


class Robot: public IterativeRobot
{
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
	DoubleSolenoid *dbSol;

	VictorSP *right;
	VictorSP *left;

	CitrusButton *gearUp;
	CitrusButton *gearDown;
	CitrusButton *forward;
	CitrusButton *backward;
	CitrusButton *single;


	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		SmartDashboard::init();

		driverL = new Joystick(0);
		driverR = new Joystick(1);
		manipulator = new Joystick(2);

		left = new VictorSP(0);
		right = new VictorSP(1);

		drivetrain = new RobotDrive(left, right);
		drivetrain->SetSafetyEnabled(false);

		elevator = new ElevatorSystem();

		compressor = new Compressor(0);

		sinSol = new Solenoid(7);
		dbSol = new DoubleSolenoid(0, 1);



		gearUp = new CitrusButton(driverL, 2);
		gearDown = new CitrusButton(driverR, 2);

		forward = new CitrusButton(manipulator, 3);
		backward = new CitrusButton(manipulator, 4);
		single = new CitrusButton(manipulator, 5);
	}

	void DisabledInit() {

		compressor->SetClosedLoopControl(false);

		if(!elevator->FullyCalibrated()){
			elevator->Reset();
		}

	}

	void DisabledPeriodic() {

		compressor->SetClosedLoopControl(false);

		if(!elevator->FullyCalibrated()){
			elevator->Calibrate();
		}

		SmartDashboard::PutNumber("Counter", static_cast<double>(elevator->elvEncoder->Get()));
		SmartDashboard::PutNumber("Avg", elevator->AvgOffset());
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		compressor->SetClosedLoopControl(true);
		//elevator->StartPIDMag(1);

	}



	void TeleopPeriodic()
	{
		compressor->SetClosedLoopControl(true);

		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);

		//elevator->MoveToMagnet(1);

		SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
		SmartDashboard::PutNumber("Avg", elevator->AvgOffset());



		if(gearUp->ButtonClicked()){
			dbSol->Set(DoubleSolenoid::Value::kReverse);
		}
		else if(gearDown->ButtonClicked()){
			dbSol->Set(DoubleSolenoid::Value::kForward);
		}
		else {

			dbSol->Set(DoubleSolenoid::Value::kOff);
		}



		UpdateButtons();

	}

	void TestPeriodic()
	{
		lw->Run();
	}


	void UpdateButtons() {
		gearDown->Update();
		gearUp->Update();

	}
};

START_ROBOT_CLASS(Robot);
