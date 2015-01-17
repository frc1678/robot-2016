#include "WPILib.h"
#include "Drivetrain.h"
#include "ElevatorSystem.h"
#include "ControlLoop.h"
#include "CitrusButton.h"

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

	Solenoid *sinSol;
	DoubleSolenoid *dbSol;

	VictorSP *right;
	VictorSP *left;

	CitrusButton *gearUp;
	CitrusButton *gearDown;
	CitrusButton *mag3;
	CitrusButton *mag4;

	bool triggered3;
	bool triggered4;

//	Talon *elv1;
//	Talon *elv2;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		SmartDashboard::init();

		driverL = new Joystick(0);
		driverR = new Joystick(1);
		manipulator = new Joystick(2);

		left = new VictorSP(1);
		right = new VictorSP(0);

		drivetrain = new RobotDrive(left, right);
		drivetrain->SetSafetyEnabled(false);

		elevator = new ElevatorSystem();

		compressor = new Compressor(0);

		sinSol = new Solenoid(7);
		dbSol = new DoubleSolenoid(0, 1);



		gearUp = new CitrusButton(driverL, 2);
		gearDown = new CitrusButton(driverR, 2);
		mag3 = new CitrusButton(manipulator, 3);
		mag4 = new CitrusButton(manipulator, 4);

//
//		elv1 = new Talon(4);
//		elv2 = new Talon(5);
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

		SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
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
		elevator->StartPIDMag(1);

		triggered3 = false;
		triggered4 = false;
	}



	void TeleopPeriodic()
	{
		compressor->SetClosedLoopControl(true);


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

//		elv1->Set(driverL->GetY());
//		elv2->Set(driverL->GetY());

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
