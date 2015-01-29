/*
 * ParametricPolynomialEquation.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Kyle
 */

#include <spline/ParametricPolynomialEquation.h>
#include <cmath>

ParametricPolynomialEquation::ParametricPolynomialEquation(
		std::vector<double> xCoefficients, std::vector<double> yCoefficients) {
	this->xcoeffs = xCoefficients;
	this->ycoeffs = yCoefficients;
}

ParametricPolynomialEquation::~ParametricPolynomialEquation() {
	if (derivative) delete derivative;
}

Point ParametricPolynomialEquation::getPoint(double t) {
	double xval = 0;
	for (unsigned int i = 0; i < xcoeffs.size(); i++) {
		xval += pow(t, (xcoeffs.size() - i - 1)) * xcoeffs[i];
	}

	double yval = 0;
	for (unsigned int i = 0; i < ycoeffs.size(); i++) {
		yval += pow(t, (ycoeffs.size() - i - 1)) * ycoeffs[i];
	}
	return Point(xval, yval);
}

void ParametricPolynomialEquation::calculateDerivative() {
	std::vector<double> nCoefficientsX;
	for (unsigned int i = 0; i < xcoeffs.size() - 1; i++) {
		nCoefficientsX.push_back(xcoeffs[i] * (xcoeffs.size() - i - 1));
	}

	std::vector<double> nCoefficientsY;
	for (unsigned int i = 0; i < ycoeffs.size() - 1; i++) {
		nCoefficientsY.push_back(ycoeffs[i] * (ycoeffs.size() - i - 1));
	}
	this->derivative =  new ParametricPolynomialEquation(nCoefficientsX, nCoefficientsY);
}
