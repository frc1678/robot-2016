
#ifndef FILESAVE_H
#define FILESAVE_H

#include "WPILib.h"

#include <iostream>
#include <fstream>
#include <ctime>
#include <sys/time.h>
#include <string>
#define LOG_STAMP "line "+to_string(__LINE__)+" in function "+__FUNCTION__+" file "+__FILE__+": "
using namespace std;

class FileSave {
	ofstream outfile;
	string filename;
	Timer *curTime;
	void logRobotInit();
	void logInit();
public:
	FileSave(string fileName);
	~FileSave();
	void reset();
	void log(string value);
	void start();
	void flush();
	void logTime();
	void logRobot(Joystick* JoystickRight, Joystick* JoystickLeft,
			Encoder* DriveEncoderRight, Encoder* DriveEncoderLeft,
			DoubleSolenoid* DriveSolenoid);
};




#endif
