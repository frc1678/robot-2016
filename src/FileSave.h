#include "WPILib.h"

#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
using namespace std;

#ifndef FILESAVE_H
#define FILESAVE_H

class FileSave {
	ofstream outfile;
	string filename;
	Timer *curTime;
	bool logging;
public:
	FileSave(string fileName);
	~FileSave();
	void reset();
	void log(string value);
	void start();
	void flush();
	void logDriveBase(Joystick* j1, Joystick* j2, Encoder* e1, Encoder* e2, DoubleSolenoid* ds);
};




#endif
