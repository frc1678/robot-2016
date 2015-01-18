/*
 * TextLogger.h
 *
 *  Created on: Jan 18, 2015
 *      Author: Citrus CAD
 */

#ifndef SRC_LOGS_TEXTLOGGER_H_
#define SRC_LOGS_TEXTLOGGER_H_
#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <sys/time.h>

#define CODE_STAMP "at line "+to_string(__LINE__)+" of "+__FILE__+" in function "+__FUNCTION__

class TextLogger {
	std::ofstream file;
	std::string filepath;
public:
	TextLogger(std::string filename);
	virtual ~TextLogger();
	void log(std::string msg, std::string cat, std::string stamp);
};

#endif /* SRC_LOGS_TEXTLOGGER_H_ */
