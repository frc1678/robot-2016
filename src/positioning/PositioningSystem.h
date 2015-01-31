/*
 * PositioningSystem.h
 *
 *  Created on: Jan 30, 2015
 *      Author: Developer
 */

#ifndef SRC_POSITIONING_POSITIONINGSYSTEM_H_
#define SRC_POSITIONING_POSITIONINGSYSTEM_H_

#include "accel/AccelInput.h"
#include "gyro/GyroReader.h"

class PositioningSystem {
protected:
	AccelInput *accel;
	GyroReader *gyro;
	GyroInterface *g;
public:
	PositioningSystem();
	double getX();
	double getY();
	double getAngle();
	void update();
	virtual ~PositioningSystem();
};

#endif /* SRC_POSITIONING_POSITIONINGSYSTEM_H_ */
