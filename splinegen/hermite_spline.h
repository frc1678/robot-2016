#ifndef SPLINE_HERMITE_SPLINE_H_
#define SPLINE_HERMITE_SPLINE_H_

#include "point.h"
#include "curve.h"

namespace spline {

class HermiteSpline : public Curve {
 public:
  HermiteSpline(Point<Length> start_pos, Point<Length> end_pos, Point<Velocity> start_vel, Point<Velocity> end_vel);
  Point<Length> CalculatePosition(double s) const override;
  Point<Velocity> CalculateVelocity(double s, double ds = 0.001) const override;
  Point<Acceleration> CalculateAcceleration(double s, double ds = 0.001) const override;
  ~HermiteSpline() {}

 private:
  Point<Unitless> a, b, c, d, e, f;
};
}

#endif /* end of include guard: SPLINE_HERMITE_SPLINE_H_ */
