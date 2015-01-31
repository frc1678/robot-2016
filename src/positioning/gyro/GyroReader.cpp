/*
 * GyroReader.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: Developer
 */

#include "GyroReader.h"

GyroReader::GyroReader(GyroInterface* g) {
	gyro = g;
	angle=0;
	gettimeofday(&time, 0);
	prevtime=time.tv_usec;
}

GyroReader::~GyroReader() {
	// TODO Auto-generated destructor stub
}

double GyroReader::update()
{
	//gets time
	double currentTime;
	gettimeofday(&time, 0);
	currentTime=time.tv_usec;

	//calculate the updated angle
	angle += (currentTime-prevtime)*gyro->ExtractAngle(0/* we should figure out what this is */);

	prevtime=currentTime;

	return angle;
}

double GyroReader::get()
{
	return angle;
}
