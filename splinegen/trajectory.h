#ifndef SPLINE_TRAJECTORY_H_
#define SPLINE_TRAJECTORY_H_

#include "muan/unitscpp/unitscpp.h"

class Trajectory {
 public:
  virtual Time GetTotalTime() = 0;

  virtual Length CalculateArcLength(Time t) = 0;
  virtual Velocity CalculateForwardVelocity(Time t) = 0;
  virtual Acceleration CalculateForwardAcceleration(Time t) = 0;

  virtual Angle CalculateHeading(Time t) = 0;
  virtual AngularVelocity CalculateAngularVelocity(Time t) = 0;
  virtual AngularAcceleration CalculateAngularAcceleration(Time t) = 0;
};

#endif /* SPLINE_TRAJECTORY_H_ */
