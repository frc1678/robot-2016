#include "FileSave.h"

FileSave::FileSave(string fileName) {
	filename = "/home/admin/logs/" + fileName;
	curTime = new Timer();
	reset();
	logInit();
	logRobotInit();
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
	strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
	timeval tv;
	gettimeofday(&tv, 0);
	int timeMS=tv.tv_usec/1000;
	char msString[25];
	sprintf(msString, "%03d", timeMS);
	outfile << buffer << '.' << msString << ",";
	/****************/
}

void FileSave::logInit()
{
	outfile.open(filename+".log", ios::trunc);
	outfile<<"last built on "<<__DATE__<<' '<<__TIME__<<"\nran at ";
	logTime();
	outfile<<"\n\n";
	outfile.close();
}
void FileSave::logRobotInit()
{
	outfile.open(filename+".csv", ios::trunc);
	logTime();
	outfile<<"JoystickRight,JoystickLeft,DriveSolenoid,"
			"DriveEncoderRightVal,DriveEncoderLeftVal,"
			"DriveEncoderRightRate,DriveEncoderLeftRate";
	outfile.close();
}

void FileSave::log(string value) {
	if(!outfile.is_open()) start();
	logTime();
	outfile << value << '\n';
}

void FileSave::start() {
	if(!outfile.is_open()) outfile.open(filename+".log", ios::app);
}

void FileSave::flush() {
	if(outfile.is_open()) outfile.close();
}

void FileSave::logRobot(Joystick* JoystickRight, Joystick* JoystickLeft,
		Encoder* DriveEncoderRight, Encoder* DriveEncoderLeft,
		DoubleSolenoid* DriveSolenoid)
{

	string to_log = to_string(JoystickRight->GetY()) + "," +
			to_string(JoystickLeft->GetY()) + "," +
			to_string(DriveSolenoid->Get()) + "," +
			to_string(DriveEncoderRight->Get()) + "," +
			to_string(DriveEncoderLeft->Get()) + "," +
			to_string(DriveEncoderRight->GetRate()) + "," +
			to_string(DriveEncoderLeft->GetRate());

	if(outfile.is_open()) outfile.close();
	outfile.open(filename+".csv", ios::app);
	logTime();
	outfile<<to_log<<'\n';
	outfile.close();
}
