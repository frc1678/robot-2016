#include "WPILib.h"
#include <fstream>
#include <iostream>
#include "ControlLoop.h"

using namespace std;

#ifndef ELEVATORSYSTEM_H
#define ELEVATORSYSTEM_H



const float ABS_ELEVATOR_POSITIONS[5] = {}; // TODO: Figure these out, right????
const float uKP1 = 0.0, uKI1 = 0.0, uKD1 = 0.0, dKP1 = 0.0, dKI1 = 0.0, dKD1 = 0.0;
const float uKP3 = 0.0, uKI3 = 0.0, uKD3 = 0.0, dKP3 = 0.0, dKI3 = 0.0, dKD3 = 0.0;
const float uKP4 = 0.0, uKI4 = 0.0, uKD4 = 0.0, dKP4 = 0.0, dKI4 = 0.0, dKD4 = 0.0;
const float uKP7 = 0.0, uKI7 = 0.0, uKD7 = 0.0, dKP7 = 0.0, dKI7 = 0.0, dKD7 = 0.0;
const float uKP8 = 0.0, uKI8 = 0.0, uKD8 = 0.0, dKP8 = 0.0, dKI8 = 0.0, dKD8 = 0.0;

const float ABS_MAG_CLICKS[6] = {-12.5, -840, -863.5, -1264.5, -1289.5, -1681.5};
const float uKP = -0.0038, uKI = -0.00025, uKD = 0,
		dKP = -0.0025, dKI = -0.001, dKD = 0.0;

const int termination = 10;

class ElevatorSystem {


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

	ControlLoop *upPIDThree;
	ControlLoop *downPIDThree;

	ControlLoop *upPIDFour;
	ControlLoop *downPIDFour;

	// THESE ARE GOING TO BE VERY SIMILAR
	ControlLoop *upPIDSeven;
	ControlLoop *downPIDSeven;

	ControlLoop *upPIDEight;
	ControlLoop *downPIDEight;

	ControlLoop *upPID;
	ControlLoop *downPID;

	double dropTime, driveClicks, driveSpeed;
	Timer* elevTimer;
	RobotDrive* drive;
	Encoder* rightEncoder, *leftEncoder;
	ControlLoop* driveStraightPID;
	PositioningSystem* pos;
public:

	bool done = false;
	Encoder *elvEncoder;

	ControlLoop *pidLoop;

	ElevatorSystem (PositioningSystem* pos, RobotDrive* drive, Encoder* right, Encoder* left);

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

	void MoveToMagnet(int magnet);

	bool AtPosition();

	void Logging (float clicks, float motorOutput);

	void MoveToStationaryPosition();

	void MoveToHPLoadOne();

	void MoveToHPLoadTwo();

	void MoveToScoringPosition();

	void MoveToGround();

	void DropStack();
};

#endif
