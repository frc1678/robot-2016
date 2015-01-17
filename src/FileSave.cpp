#include "FileSave.h"

FileSave::FileSave(string fileName) {
	filename = "/home/admin/logs/" + fileName;
	curTime = new Timer();
	curTime->Reset();
	curTime->Start();
}
FileSave::~FileSave() {

}

void FileSave::reset() {
	curTime->Reset();
	curTime->Start();
}

void FileSave::logTime()
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

void FileSave::logRobotInit()
{
	log("Joystick1,Joystick2,DriveSolenoid,DriveEncoder1Val,DriveEncoder2Val,DriveEncoder1Rate,DriveEncoder2Rate");
}

void FileSave::log(string value) {
	logTime();
	outfile << value << '\n';
}
	
void FileSave::start() {
	outfile.open(filename, ios::app);
}

void FileSave::flush() {
	outfile.close();
}
void FileSave::logRobot(Joystick* j1, Joystick* j2, Encoder* e1, Encoder* e2, DoubleSolenoid* ds)
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