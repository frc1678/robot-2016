/*
 * ParametricPolynomialEquation.h
 *
 *  Created on: Jan 20, 2015
 *      Author: Kyle
 */

#ifndef SRC_SPLINE_PARAMETRICPOLYNOMIALEQUATION_H_
#define SRC_SPLINE_PARAMETRICPOLYNOMIALEQUATION_H_

#include <spline/Equation.h>
#include <vector>

class ParametricPolynomialEquation: public Equation {
	std::vector<double> xcoeffs, ycoeffs;
public:
	void calculateDerivative() override;
	ParametricPolynomialEquation(std::vector<double> xCoefficients,
			std::vector<double> yCoefficients);
	virtual ~ParametricPolynomialEquation();
	Point getPoint(double t) override;
};

#endif /* SRC_SPLINE_PARAMETRICPOLYNOMIALEQUATION_H_ */
