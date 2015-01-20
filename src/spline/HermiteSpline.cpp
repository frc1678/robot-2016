/*
 * HermiteSpline.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <spline/HermiteSpline.h>
#include <cmath>

HermiteSpline::HermiteSpline(Point startPos, Point startVel, Point startAcc, Point endPos, Point endVel, Point endAcc) {
	// TODO Auto-generated constructor stub

}

HermiteSpline::~HermiteSpline() {
	// TODO Auto-generated destructor stub
}

Point HermiteSpline::getPoint(double t)
{
	return Point(a.x * pow(t, 5) + b.x * pow(t, 4) + c.x * pow(t, 3) + d.x * pow(t, 2) + e.x * t + f.x,
			a.y * pow(t, 5) + b.y * pow(t, 4) + c.y * pow(t, 3) + d.y * pow(t, 2) + e.y * t + f.y);
}

Equation* HermiteSpline::getDerivative()
{

}
