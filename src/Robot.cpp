#include "WPILib.h"
#include "Drivetrain.h"
#include "ElevatorSystem.h"
#include "ControlLoop.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;

	Joystick *driverL;
	Joystick *driverR;

	RobotDrive *drivetrain;

	ElevatorSystem *elevator;



	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		SmartDashboard::init();

		driverL = new Joystick(0);
		driverR = new Joystick(1);

		drivetrain = new RobotDrive(3, 1);
		drivetrain->SetSafetyEnabled(false);

		elevator = new ElevatorSystem();

	}

	void DisabledInit() {
		if(!elevator->FullyCalibrated()){
			elevator->Reset();
		}
		//elevator->Reset();

	}

	void DisabledPeriodic() {
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
		elevator->StartPIDMag(1);

	}

	void TeleopPeriodic()
	{
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);

		elevator->MoveToMagnet(1);

		SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
		SmartDashboard::PutNumber("Avg", elevator->AvgOffset());

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
