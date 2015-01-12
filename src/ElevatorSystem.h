#include "WPILib.h"
#include <fstream>
#include <iostream>
#include "ControlLoop.h"

using namespace std;


const float ABS_MAG_CLICKS[6] = {-12.5, -840, -863.5, -1264.5, -1289.5, -1681.5};
const float uKP = -0.0038, uKI = -0.00025, uKD = 0, dKP = -0.0032, dKI = -0.0013, dKD = 0.0;

const int termination = 10;

class ElevatorSystem {

	//Preferences *prefs;

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

	void StartPID(int encoderPos);

	void StopPID();

	void MoveToAbsEncoderPosition(int encoderPos);

	void MoveToMagnet(int magnet);

	void Logging (float clicks, float motorOutput);

};
