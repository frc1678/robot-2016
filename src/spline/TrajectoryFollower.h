/*
 * TrajectoryFollower.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#ifndef SRC_SPLINE_TRAJECTORYFOLLOWER_H_
#define SRC_SPLINE_TRAJECTORYFOLLOWER_H_

#include "HermiteSpline.h"

class TrajectoryFollower {
	Equation* toFollow;
	double progress;
	double angle, speed;
	void updateBearings(double dt);
	void move();
public:
	TrajectoryFollower(Equation* toFollow);
	void run(double dt);
	virtual ~TrajectoryFollower();
};

#endif /* SRC_SPLINE_TRAJECTORYFOLLOWER_H_ */
