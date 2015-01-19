/*
 * CSVLogger.h
 *
 *  Created on: Jan 18, 2015
 *      Author: Citrus CAD
 */

#ifndef SRC_LOGS_CSVLOGGER_H_
#define SRC_LOGS_CSVLOGGER_H_

#include <iostream>
#include <fstream>
#include <ctime>
#include <sys/time.h>
#include <string>

class CSVLogger {
	std::ofstream file;
	std::string filepath;
public:
	CSVLogger(std::string filename, std::string varnames);
	virtual ~CSVLogger();
	void LogValue(std::string val);
	void StartNewCycle();
};

#endif /* SRC_LOGS_CSVLOGGER_H_ */