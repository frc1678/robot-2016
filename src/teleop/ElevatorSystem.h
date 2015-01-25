#include "WPILib.h"
#include <fstream>
#include <iostream>
#include "ControlLoop.h"

using namespace std;

#ifndef ELEVATORSYSTEM_H
#define ELEVATORSYSTEM_H



const float ABS_ELEVATOR_POSITIONS[5] = {}; // TODO: Figure these out
const float ABS_MAG_CLICKS[6] = {-12.5, -840, -863.5, -1264.5, -1289.5, -1681.5};
const float uKP = -0.0038, uKI = -0.00025, uKD = 0,
		dKP = -0.0025, dKI = -0.001, dKD = 0.0;

const int termination = 10;

class ElevatorSystem {


	bool oldstate;
	bool goingDown = false;
	bool done = false;

	float n;
	float sum;
	float offset;

	int magClicksIndex;

	float clicks;

	ofstream encoderErrorFile;
	ofstream motorOutputErrorFile;

	DigitalInput *hallSensor;
	Talon *left;
	Talon *right;

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
	ControlLoop *pidLoop;

public:

	Encoder *elvEncoder;


	ElevatorSystem ();

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

};

#endif
