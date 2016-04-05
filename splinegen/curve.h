#ifndef SPLINE_CURVE_H_
#define SPLINE_CURVE_H_

#include "point.h"

namespace spline {
class Curve {
 public:
  virtual Point<Length> CalculatePosition(double p) const = 0;
  virtual Point<Velocity> CalculateVelocity(double p, double dp = 0.001) const;
  virtual Point<Acceleration> CalculateAcceleration(double p, double dp = 0.001) const;

  virtual Length CalculateArcLength(double p0, double p1, double dp = 0.001) const;

  virtual Angle CalculateHeading(double p, double dp = 0.001) const;
  
  virtual Velocity CalculateForwardVelocity(double p, double dp = 0.001) const;
  virtual AngularVelocity CalculateAngularVelocity(double p, double dp = 0.001) const;

  virtual Acceleration CalculateForwardAcceleration(double p, double dp = 0.001) const;
  virtual AngularAcceleration CalculateAngularAcceleration(double p, double dp = 0.001) const;

  virtual ~Curve() {}
};
}

#endif /* end of include guard: SPLINE_CURVE_H_ */
