

#ifndef ROBOT_H
#define ROBOT_H

#include <logs/ConstantsLoader.h>
#include "WPILib.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace std;
class Robot: public IterativeRobot
{
private:
	friend class FileSave;

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


	ConstantsLoader* kLoad;

	void RobotInit();

	void AutonomousInit();

	void AutonomousPeriodic();

	void TeleopInit();

	void TeleopPeriodic();

	void TestPeriodic();

	void DisabledInit();

	void DisabledPeriodic();

	void UpdateButtons();
};

#endif
