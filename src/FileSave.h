#include "WPILib.h"

#include <iostream>
#include <fstream>
#include <ctime>
#include <sys/time.h>
#include <string>
using namespace std;

#ifndef FILESAVE_H
#define FILESAVE_H

class FileSave {
	ofstream outfile;
	string filename;
	Timer *curTime;
public:
	FileSave(string fileName){
		filename = "/home/admin/logs/" + fileName;
		curTime = new Timer();
		curTime->Reset();
		curTime->Start();
	}
	~FileSave(){}

	void reset(){
		curTime->Reset();
		curTime->Start();
	}

	void log(string value) {
		logTime();
		outfile << value << '\n';
	}
	void start()
	{
		outfile.open(filename, ios::app);
	}
	void flush()
	{
		outfile.close();
	}
	void logTime()
	{
		/**LOG THE TIME**/
		time_t rawtime;
		tm* timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
		timeval tv;
		gettimeofday(&tv, NULL);
		int timeMS=tv.tv_usec/1000;
		char msString[25];
		sprintf(msString, "%03d", timeMS);
		outfile << buffer << '.' << msString << ",";
		/****************/
	}
	void logRobotInit()
	{
		log("Joystick1,Joystick2,DriveSolenoid,DriveEncoder1Val,DriveEncoder2Val,DriveEncoder1Rate,DriveEncoder2Rate");
	}
	void logRobot(Joystick* j1, Joystick* j2, Encoder* e1, Encoder* e2, DoubleSolenoid* ds)
	{

		string to_log = to_string(j1->GetY()) + "," +
						to_string(j2->GetY()) + "," +
						to_string(ds->Get()) + "," +
						to_string(e1->Get()) + "," +
						to_string(e2->Get()) + "," +
						to_string(e1->GetRate()) + "," +
						to_string(e2->GetRate());
		log(to_log);
	}
};




#endif
