/*
 * HermiteSpline.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#ifndef SRC_SPLINE_HERMITESPLINE_H_
#define SRC_SPLINE_HERMITESPLINE_H_
#include "Equation.h"
class HermiteSpline : Equation {
	Point a, b, c, d, e, f;
public:
	HermiteSpline(Point startPos, Point startVel, Point startAcc, Point endPos, Point endVel, Point endAcc);
	virtual ~HermiteSpline();
	virtual Point getPoint(double t) override;
	virtual Equation* getDerivative() override;
};

#endif /* SRC_SPLINE_HERMITESPLINE_H_ */
