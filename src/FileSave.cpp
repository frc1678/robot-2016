#include "FileSave.h"

FileSave::FileSave(string fileName) {
	logging = false;
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

void FileSave::log(string value) {
	if (!logging) {
		logging = true;
		outfile.is_open();
		outfile << value << endl;
	}
	outfile << value << endl;
}
void FileSave::start() {
	logging = true;
	outfile.open(filename, ios::app);
}
void FileSave::flush() {
	logging = false;
	outfile.close();
}
void FileSave::logDriveBase(Joystick* j1, Joystick* j2, Encoder* e1,
		Encoder* e2, DoubleSolenoid* ds) {
	start();
	log("-------------------------");
	log("Joystick 1 Value: " + to_string(j1->GetY()));
	log("Joystick 2 Value: " + to_string(j2->GetY()));
	log("Solenoid Value: " + to_string(ds->Get()));
	log("Encoder 1 Value: " + to_string(e1->Get()));
	log("Encoder 2 Value: " + to_string(e2->Get()));
	log("Encoder 1 Rate: " + to_string(e1->GetRate()));
	log("Encoder 2 Rate: " + to_string(e2->GetRate()));
	flush();
}
