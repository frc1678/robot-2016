/*
 * Equation.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#ifndef SRC_SPLINE_EQUATION_H_
#define SRC_SPLINE_EQUATION_H_

struct Point {
	double x, y;
	Point(double nX, double nY) :
			x(nX), y(nY) {
	}
	Point() :
			x(0), y(0) {
	}
};

class Equation {
protected:
	Equation* derivative;
	virtual void calculateDerivative() = 0;
public:
	Equation() {}
	virtual ~Equation() {}
	virtual Point getPoint(double t) = 0;
	virtual Equation* getDerivative() {
		if (!derivative)
			calculateDerivative();
		return derivative;
	}
};

#endif /* SRC_SPLINE_EQUATION_H_ */
