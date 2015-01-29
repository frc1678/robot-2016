/*
 * GyroReader.h
 *
 *  Created on: Jan 29, 2015
 *      Author: Developer
 */

#ifndef SRC_TELEOP_GYRO_GYROREADER_H_
#define SRC_TELEOP_GYRO_GYROREADER_H_

#include "gyro_interface.h"
#include <sys/time.h>

class GyroReader {
public:
	GyroReader(GyroInterface* g);
	double update();
	double get();
	virtual ~GyroReader();
protected:
	double angle;
	GyroInterface *gyro;
	timeval time;
	double prevtime;
};

#endif /* SRC_TELEOP_GYRO_GYROREADER_H_ */
