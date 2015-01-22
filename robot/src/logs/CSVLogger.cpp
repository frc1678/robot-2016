/*
 * CSVLogger.cpp
 *
 *  Created on: Jan 18, 2015
 *      Author: Citrus CAD
 */

#include "CSVLogger.h"

CSVLogger::CSVLogger(std::string filename, std::string varnames) {
	file.open(filepath = ("/home/admin/logs/" + filename + ".csv"));
	file << varnames << std::endl;
	StartNewCycle();
}

CSVLogger::~CSVLogger() {
	file.close();
}

void CSVLogger::LogValue(std::string val)
{
	file << val << ",";
}

void CSVLogger::StartNewCycle()
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

	file << std::endl << buffer << '.' << msString << ",";
}
