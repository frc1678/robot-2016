/*
 * Equation.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#ifndef SRC_SPLINE_EQUATION_H_
#define SRC_SPLINE_EQUATION_H_

struct Point
{
	double x, y;
	Point(double nX, double nY) : x(nX), y(nY) {}
};

class Equation {
public:
	Equation();
	virtual ~Equation();
	virtual Point getPoint(double t) = 0;
	virtual Equation* getDerivative() = 0;
};

#endif /* SRC_SPLINE_EQUATION_H_ */
