#include "WPILib.h"
#include "Drivetrain.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;

	Joystick *driverL;
	Joystick *driverR;

	RobotDrive *drivetrain;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		driverL = new Joystick(0);
		driverR = new Joystick(1);

		drivetrain = new RobotDrive(3, 1);
		drivetrain->SetSafetyEnabled(false);

	}

	void DisabledInit() {

	}

	void DisabledPeriodic() {

	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
