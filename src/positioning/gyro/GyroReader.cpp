/*
 * GyroReader.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: Developer
 */

#include "GyroReader.h"
#include "WPILib.h"
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
	uint32_t reading = gyro->GetReading();
	angle += (currentTime-prevtime)*gyro->ExtractAngle(reading);

	prevtime=currentTime;
	SmartDashboard::PutNumber("GyroAngle", angle);
	return angle;
}

double GyroReader::get()
{
	return angle;
}
