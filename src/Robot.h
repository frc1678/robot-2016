

#ifndef ROBOT_H
#define ROBOT_H

#include "WPILib.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "FileSave.h"
#include "ConstantsLoader.h"
using namespace std;
class Robot: public IterativeRobot
{
private:
	friend class FileSave;
	LiveWindow *lw;
	RobotDrive *drivetrain;
	ConstantsLoader* kLoad;
	ConstantsLoader* kAuto;
	Joystick *rightJoystick;
	Joystick *leftJoystick;
	DoubleSolenoid *ds;

	Encoder *rightEncoder;
	Encoder *leftEncoder;
	FileSave *fsave;

	void RobotInit();

	void AutonomousInit();

	void AutonomousPeriodic();

	void TeleopInit();

	void TeleopPeriodic();

	void TestPeriodic();

	void DisabledInit();
};

START_ROBOT_CLASS(Robot);

#endif
