/*
 * FileLooker.h
 *
 *  Created on: Jan 11, 2015
 *      Author: Citrus Circuits
 */

#ifndef ROBOT_2015_SRC_LOGS_CONSTANTSLOADER_H_
#define ROBOT_2015_SRC_LOGS_CONSTANTSLOADER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace std;

class ConstantsLoader {
	ifstream constantsFile;
	map<string, double> constantsMap;
	string filepath;
public:
	ConstantsLoader(string file = "coefficients.txt");
	virtual ~ConstantsLoader();
	void reload();
	double getConstant(string n, double defaultVal = 0);
};

#endif /* ROBOT_2015_SRC_LOGS_CONSTANTSLOADER_H_ */
