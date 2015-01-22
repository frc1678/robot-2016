

#ifndef ROBOT_H
#define ROBOT_H

//#include <logs/ConstantsLoader.h>
#include "WPILib.h"
//#include "logs/ConstantsLoader.h"
#include "teleop/CitrusButton.h"
#include "teleop/ElevatorSystem.h"
#include "teleop/Drivetrain.h"
//#include <iostream>
//#include <fstream>
//#include <stdlib.h>

//using namespace std;

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "teleop/SteeringWheelDrive.h"
#include "logs/CSVLogger.h"
#include "logs/TextLogger.h"
#include "logs/BunchLogger.h"

using namespace std;


class Robot: public IterativeRobot
{

public:


	LiveWindow *lw;

	// Joysticks that Takumi drives with
	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator;

	Joystick* steeringWheel;
	Joystick* speedJoystick;

	// For the driving of the robot
	RobotDrive *drivetrain;
	SteeringWheelDrive* swd;

	// The elevator
	ElevatorSystem *elevator;

	Compressor *compressor;

	DoubleSolenoid *shifting;


	CitrusButton *gearUp;
	CitrusButton *gearDown;

//	CitrusButton *mag3;
//	CitrusButton *mag4;
//
//	bool triggered3;
//	bool triggered4;

	CitrusButton *mag3;
	CitrusButton *mag4;
	CitrusButton *SteeringWheelChoice;





	//ConstantsLoader* kLoad;

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
