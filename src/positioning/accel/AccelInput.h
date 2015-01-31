/*
 * AccelInput.h
 *
 *  Created on: Jan 30, 2015
 *      Author: Developer
 */

#ifndef SRC_ACCEL_ACCELINPUT_H_
#define SRC_ACCEL_ACCELINPUT_H_

#include <sys/time.h>
#include <math.h>
#include "WPILib.h"
class AccelInput {
protected:
	timeval *timer;
	BuiltInAccelerometer *accel;
	double xPos, yPos;
	double xVel, yVel;
	double prevTime;
public:
	AccelInput();
	double getAccel(char axis);
	double getX();
	double getY();
	void update(double angle);
	virtual ~AccelInput();
};

#endif /* SRC_ACCEL_ACCELINPUT_H_ */
