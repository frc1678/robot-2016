#include "WPILib.h"
#include <fstream>
#include <iostream>
#include "ControlLoop.h"
#include "positioning/PositioningSystem.h"
#include "logs/ConstantsLoader.h"

using namespace std;

#ifndef ELEVATORSYSTEM_H
#define ELEVATORSYSTEM_H

/*
 *
 */

const float ABS_ELEVATOR_POSITIONS[7] = {-20, -115, -561, -822, -1484, -2863, -3201}; // TODO: Figure these out, right????

// TOP VALUE IS -3397
const float ABS_MAG_CLICKS[2] = {-1625, -1684.0}; //TODO first when testing elevator
const float uKP = -0.0038, uKI = -0.00025, uKD = 0,
		dKP = -0.0025, dKI = -0.001, dKD = 0.0;

const int termination = 10;

class ElevatorSystem {

	float uKP1 = 0.0, uKI1 = 0.0, uKD1 = 0.0, dKP1 = 0.0, dKI1 = 0.0, dKD1 = 0.0;
	float uKP2 = 0.0, uKI2 = 0.0, uKD2 = 0.0, dKP2 = 0.0, dKI2 = 0.0, dKD2 = 0.0;
	float uKP3 = 0.0, uKI3 = 0.0, uKD3 = 0.0, dKP3 = 0.0, dKI3 = 0.0, dKD3 = 0.0;
	float uKP4 = 0.0, uKI4 = 0.0, uKD4 = 0.0, dKP4 = 0.0, dKI4 = 0.0, dKD4 = 0.0;
	float uKP5 = 0.0, uKI5 = 0.0, uKD5 = 0.0, dKP5 = 0.0, dKI5 = 0.0, dKD5 = 0.0;
	float uKP6 = 0.0, uKI6 = 0.0, uKD6 = 0.0, dKP6 = 0.0, dKI6 = 0.0, dKD6 = 0.0;
	float uKP7 = 0.0, uKI7 = 0.0, uKD7 = 0.0, dKP7 = 0.0, dKI7 = 0.0, dKD7 = 0.0;

	bool oldstate;
	bool goingDown = false;


	float n;
	float sum;
	float offset;

	int magClicksIndex;

	float clicks;

	ofstream encoderErrorFile;
	ofstream motorOutputErrorFile;

	DigitalInput *hallSensor;
	VictorSP *elvMotor;

	ControlLoop *upPIDOne;
	ControlLoop *downPIDOne;

	ControlLoop *upPIDTwo;
	ControlLoop *downPIDTwo;

	ControlLoop *upPIDThree;
	ControlLoop *downPIDThree;

	ControlLoop *upPIDFour;
	ControlLoop *downPIDFour;

	ControlLoop *upPIDFive;
	ControlLoop *downPIDFive;

	ControlLoop *upPIDSix;
	ControlLoop *downPIDSix;

	ControlLoop *upPIDSeven;
	ControlLoop *downPIDSeven;



	ControlLoop *upPID;
	ControlLoop *downPID;

	double dropTime, driveClicks, driveSpeed;
	Timer* elevTimer;

	RobotDrive* drive;
	Encoder* rightEncoder, *leftEncoder;

	ControlLoop* driveStraightPID;
	PositioningSystem* pos;
	Solenoid *diskBreak;


public:

	bool done = false;
	Encoder *elvEncoder;

	ControlLoop *pidLoop;
	ConstantsLoader *kLoad;


	ElevatorSystem (PositioningSystem* pos, RobotDrive* drive, Solenoid *db, Encoder* right, Encoder* l);

	~ElevatorSystem();


	void Calibrate();

	float AvgOffset ();

	float FindOffset (float location, float current);

	bool HallTriggered ();

	void Reset();

	bool FullyCalibrated ();

	void StartPIDMag(int mag);

	void StartPIDPosition(int pos);

	void StartPID(int encoderPos);

	void StopPID();

	void MoveToAbsEncoderPosition(int encoderPos);

	bool AtPosition();

	void Logging (float clicks, float motorOutput);

	void DropStack();

	void MoveElevator(float motorInput);

	void ReloadConstants();

	void SetDiskBreak(bool on);


	// State machine movement positions:
	void MoveTo_One_PickupRC();
	void MoveTo_Two_BackupFromStack();
	void MoveTo_Three_PrepStackPickup();
	void MoveTo_Four_HoldStack();
	void MoveTo_Five_PrepHPOne();
	void MoveTo_Six_HPWaitRC();
	void MoveTo_Seven_HPWaitTote();
};

#endif
