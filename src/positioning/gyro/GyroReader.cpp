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
	counter = 0;
	gyroPos = 0.0;
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
	gyroPos = gyro->ExtractAngle(reading);
	if( gyroPos > 0.01 || gyroPos < -0.01){
		angle += gyro->ExtractAngle(reading);
	}

	if((counter % 25) == 0){
		prevtime=currentTime;
		SmartDashboard::PutNumber("GyroAngleDelta", gyro->ExtractAngle(reading));
		SmartDashboard::PutNumber("GyroAngle", angle);
		SmartDashboard::PutNumber("Counter", counter);
	}
	counter++;
	return angle;
}

double GyroReader::get()
{
	return angle;
}
