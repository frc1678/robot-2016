/*
 * FileLooker.h
 *
 *  Created on: Jan 11, 2015
 *      Author: Citrus Circuits
 */

#ifndef SRC_CONSTANTSLOADER_H_
#define SRC_CONSTANTSLOADER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace std;

class ConstantsLoader {
	ifstream constantsFile;
	map<string, double> constantsMap;
	string file;
public:
	ConstantsLoader(string filepath = "home/admin/constants/coefficients.txt");
	virtual ~ConstantsLoader();
	void reload();
	double getConstant(string n, double defaultVal = 0);
};

#endif /* SRC_CONSTANTSLOADER_H_ */
