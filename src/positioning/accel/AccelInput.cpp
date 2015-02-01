/*
 * AccelInput.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Developer
 */

#include "AccelInput.h"

AccelInput::AccelInput() {
	accel = new BuiltInAccelerometer();
	xPos=0; yPos=0;
	xVel=0; yVel=0;
	timer = new timeval();
	gettimeofday(timer, 0);
	prevTime=timer->tv_usec;
}

AccelInput::~AccelInput() {
	delete accel;
}

void AccelInput::update(double angle)
{
	//update the time
	double currentTime;
	gettimeofday(timer, 0);
	currentTime=timer->tv_usec;

	//calculate the relative increase in velocity
	double xVelStep = (currentTime-prevTime)*accel->GetX();
	double yVelStep = (currentTime-prevTime)*accel->GetY();

	//update velocity
	xVel+=xVelStep*cos(angle)-yVelStep*sin(angle);
	yVel+=yVelStep*cos(angle)+xVelStep*sin(angle);

	//update position
	xPos+=(currentTime-prevTime)*xVel;
	yPos+=(currentTime-prevTime)*yVel;

	prevTime=currentTime;
}

double AccelInput::getX()
{
	return xPos;
}

double AccelInput::getY()
{
	return yPos;
}
