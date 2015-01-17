/*
 * FileLooker.cpp
 *
 *  Created on: Jan 11, 2015
 *      Author: Citrus Circuits
 */

#include <ConstantsLoader.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

using namespace std;

ConstantsLoader::ConstantsLoader(string filepath) {
	constantsFile.open(filepath);
	string key;
	double value;
	while (constantsFile >> key >> value) {
		constantsMap[key] = value;
	}

	constantsFile.close();
}
ConstantsLoader::~ConstantsLoader() {}

double ConstantsLoader::getConstant(string key, double defaultValue){
	if (constantsMap.count(key) != 0){
		return constantsMap[key];
	}
	else
	{
		return defaultValue;
	}
}

void ConstantsLoader::reload()
{
	constantsFile.open(file);
	string key;
	double value;
	while (constantsFile >> key >> value) {
		constantsMap[key] = value;
	}

	constantsFile.close();
}
