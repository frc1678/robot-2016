

#ifndef ROBOT_H
#define ROBOT_H

//#include <logs/ConstantsLoader.h>
#include "WPILib.h"
//#include "logs/ConstantsLoader.h"
#include "teleop/CitrusButton.h"
#include "teleop/ElevatorSystem.h"
#include "teleop/Drivetrain.h"
#include "teleop/PincherSystem.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <auto/AutonomousRoutine.h>

using namespace std;

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "teleop/SteeringWheelDrive.h"
#include "logs/CSVLogger.h"
#include "logs/TextLogger.h"
#include <logs/SystemLogger.h>

using namespace std;


class Robot: public IterativeRobot
{

public:
	AutonomousRoutine* autoCode;

	LiveWindow *lw;

	// Joysticks that Takumi drives with
	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator;

	//for wheel drive
//	Joystick* steeringWheel;
//	Joystick* speedJoystick;

	// For the driving of the robot
	RobotDrive *drivetrain;
	SteeringWheelDrive* swd;

	// The elevator
	ElevatorSystem *elevator;

	Compressor *compressor;

	DoubleSolenoid *shifting;


	CitrusButton *gearUp;
	CitrusButton *gearDown;

	CitrusButton* straightButton;

//	CitrusButton *mag3;
//	CitrusButton *mag4;
//
//	bool triggered3;
//	bool triggered4;

	CitrusButton *mag3;
	CitrusButton *mag4;
	CitrusButton *SteeringWheelChoice;
	CitrusButton *runPinchers;
	CitrusButton *reversePinchers;
	CitrusButton *openPinchers;
	CitrusButton *closePinchers;

	PincherSystem *pinchers;

	CSVLogger *ElevLog;

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
