/*
 * CSVLogger.h
 *
 *  Created on: Jan 18, 2015
 *      Author: Citrus CAD
 */

#ifndef ROBOT_2015_SRC_LOGS_CSVLOGGER_H_
#define ROBOT_2015_SRC_LOGS_CSVLOGGER_H_

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

#endif /* ROBOT_2015_SRC_LOGS_CSVLOGGER_H_ */
