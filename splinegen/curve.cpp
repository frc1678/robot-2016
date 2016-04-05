#include "curve.h"
#include "muan/unitscpp/unitscpp.h"
#include "point.h"
#include <cmath>

namespace spline {

Point<Velocity> Curve::CalculateVelocity(double p, double dp) const {
  return (CalculatePosition(p + dp / 2) - CalculatePosition(p - dp / 2)) / (dp * s);
}

Point<Acceleration> Curve::CalculateAcceleration(double p, double dp) const {
  return (CalculateVelocity(p + dp / 2) - CalculateVelocity(p - dp / 2)) / (dp * s);
}

Length Curve::CalculateArcLength(double p0, double p1, double dp) const {
  Length arc_length = 0.0;
  for (double p = p0; p < p1; p += dp) {
    arc_length += (CalculatePosition(p + dp) - CalculatePosition(p)).Magnitude();
  }
  return arc_length;
}

Angle Curve::CalculateHeading(double p, double dp) const {
  return CalculateVelocity(p, dp).Direction();
}

Velocity Curve::CalculateForwardVelocity(double p, double dp) const {
  return CalculateVelocity(p, dp).Magnitude();
}

AngularVelocity Curve::CalculateAngularVelocity(double p, double dp) const {
  return (CalculateHeading(p + dp) - CalculateHeading(p)) / (dp * s);
}

Acceleration Curve::CalculateForwardAcceleration(double p, double dp) const {
  return CalculateAcceleration(p, dp).Magnitude();
}

AngularAcceleration Curve::CalculateAngularAcceleration(double p, double dp) const {
  return (CalculateAngularVelocity(p + dp, dp) - CalculateAngularVelocity(p, dp)) / (dp * s);
}

}
