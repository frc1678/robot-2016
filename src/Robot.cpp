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

	VictorSP *vic;

	CitrusButton *gearUp;
	CitrusButton *gearDown;
	CitrusButton *trigger;



	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		SmartDashboard::init();

		driverL = new Joystick(0);
		driverR = new Joystick(1);
		manipulator = new Joystick(2);

		drivetrain = new RobotDrive(3, 1);
		drivetrain->SetSafetyEnabled(false);

		elevator = new ElevatorSystem();

		compressor = new Compressor(0);

		sinSol = new Solenoid(7);
		dbSol = new DoubleSolenoid(0, 1);

		vic = new VictorSP(0);

		gearUp = new CitrusButton(manipulator, 1);
		gearDown = new CitrusButton(manipulator, 2);
		trigger = new CitrusButton(manipulator, 3);
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
		//elevator->StartPIDMag(1);

	}

	void TeleopPeriodic()
	{
		compressor->SetClosedLoopControl(true);

		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);

		//elevator->MoveToMagnet(1);

		SmartDashboard::PutNumber("Counter", elevator->elvEncoder->Get());
		SmartDashboard::PutNumber("Avg", elevator->AvgOffset());


	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
