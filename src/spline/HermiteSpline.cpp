/*
 * HermiteSpline.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#include <spline/HermiteSpline.h>
#include <cmath>
#include <vector>
#include "ParametricPolynomialEquation.h"

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

void HermiteSpline::calculateDerivative()
{
	if (derivative) delete derivative;
	std::vector<double> coefficientsX;
	coefficientsX.push_back(e.x);
	coefficientsX.push_back(2*d.x);
	coefficientsX.push_back(3*c.x);
	coefficientsX.push_back(4*b.x);
	coefficientsX.push_back(5*a.x);

	std::vector<double> coefficientsY;
	coefficientsY.push_back(e.x);
	coefficientsY.push_back(2*d.x);
	coefficientsY.push_back(3*c.x);
	coefficientsY.push_back(4*b.x);
	coefficientsY.push_back(5*a.x);

	this->derivative = new ParametricPolynomialEquation(coefficientsX, coefficientsY);
}
