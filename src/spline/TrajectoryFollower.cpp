/*
 * TrajectoryFollower.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <spline/TrajectoryFollower.h>
#include <cmath>

double findAngle(Point p)
{
	return atan2(p.y, p.x);
}
double findSpeed(Point p)
{
	return sqrt(p.x*p.x + p.y*p.y);
}

TrajectoryFollower::TrajectoryFollower(Equation* toFollow) {
	progress = 0;
	this->toFollow = toFollow;
	Point diff = toFollow->getPoint(progress);
	diff.x -= toFollow->getPoint(progress + .001).x;
	diff.y -= toFollow->getPoint(progress + .001).y;
	angle = findAngle(diff);
	speed = findSpeed(diff);
}

TrajectoryFollower::~TrajectoryFollower() {
	// TODO Auto-generated destructor stub
}

void TrajectoryFollower::updateBearings(double dt)
{
	Point diff = toFollow->getPoint(progress);
	diff.x -= toFollow->getPoint(progress + dt).x;
	diff.y -= toFollow->getPoint(progress + dt).y;
	angle = findAngle(diff);
	speed = findSpeed(diff);
}

void TrajectoryFollower::run(double dt)
{
	progress += dt;
	//progress = 1;
	updateBearings(dt);
	//move();
}
