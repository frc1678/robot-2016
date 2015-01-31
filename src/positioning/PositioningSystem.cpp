/*
 * PositioningSystem.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Developer
 */

#include "PositioningSystem.h"
#include "gyro/GyroReader.h"

PositioningSystem::PositioningSystem() {
	g = new GyroInterface();
	gyro = new GyroReader(g);
	accel = new AccelInput();
}

PositioningSystem::~PositioningSystem() {
	delete g;
	delete gyro;
	delete accel;
}

void PositioningSystem::update()
{
	accel->update(gyro->update());
}

double PositioningSystem::getAngle()
{
	return gyro->get();
}

double PositioningSystem::getX()
{
	return accel->getX();
}

double PositioningSystem::getY()
{
	return accel->getY();
}
