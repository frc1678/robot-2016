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

	//calculate the updated positions
	double xPosRel = (currentTime-prevTime)*accel->GetX();
	double yPosRel = (currentTime-prevTime)*accel->GetY();

	prevTime=currentTime;

	//do calculations
	xPos+=xPosRel*cos(angle)-yPosRel*sin(angle);
	yPos+=yPosRel*cos(angle)+xPosRel*sin(angle);
}

double AccelInput::getX()
{
	return xPos;
}

double AccelInput::getY()
{
	return yPos;
}
