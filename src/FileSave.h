#include "WPILib.h"

#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
using namespace std;

#ifndef FILESAVE_H
#define FILESAVE_H

class FileSave {
	
	ofstream outflash;
	fstream outfile;
	bool isLogging;
	Timer *curTime;
public:
	FileSave(){
		
		curTime = new Timer();
		curTime->Reset();
		curTime->Start();
	}
	~FileSave(){}
	
	void reset(){
		curTime->Reset();
		curTime->Start();
	}
	
	void logfile(Encoder *right_encoder, Encoder *left_encoder){//sends encoders as parameters
		outfile.open("EncoderSaver.txt");
		outfile << "Time: " << curTime->Get() << endl;
		outfile << "Right Encoder: " << right_encoder->GetDistance() << endl;
		outfile << "Left Encoder: " << left_encoder->GetDistance() << endl; //pass through encoders and log their values
		outfile.close();
	}
	void saveFlash(Encoder *right_encoder, Encoder *left_encoder) {// should save data to flash drive
		outflash.open(/*directory*/"D:\\EncoderFile.txt");
		outflash << "Time: " << curTime->Get() << endl;
		outflash << "Right Encoder: " << right_encoder->GetDistance() << endl;
		outflash << "Left Encoder: " << left_encoder->GetDistance() << endl;
		outflash.close();
	}
	string getfile(string filename){
		string line;
		outfile.open(filename + ".txt");
		while (getline (outfile, line)){
			 return line;
		}

		outfile.close();
	}
	void log(string value) {
		outfile.open("/home/admin/logs/log.log", ios::app);
		outfile << value << endl;
		outfile.close();
	}
};


#endif FILESAVE_H
