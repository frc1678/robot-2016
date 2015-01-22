/*
 * TextLogger.cpp
 *
 *  Created on: Jan 18, 2015
 *      Author: Citrus CAD
 */

#include "TextLogger.h"

TextLogger::TextLogger(std::string filename) {
	file.open(filepath = ("/home/admin/logs/" + filename + ".log"));
	file << "last built on " << __DATE__ << " at " << __TIME__ << std::endl;
}

TextLogger::~TextLogger() {
	file.close();
}

void TextLogger::TextLog(std::string msg, std::string cat, std::string codestamp)
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
	/****************/

	// [PID][2015-3-23 10:55:40.912] Elevator did things (at line 352 in Elevator.cpp)
	file << '[' << cat <<  "][" << buffer << '.' << msString << "] " << msg << " (" << codestamp << ")"<< std::endl;
}
