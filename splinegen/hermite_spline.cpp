#include "hermite_spline.h"
#include "muan/unitscpp/unitscpp.h"

namespace spline {

HermiteSpline::HermiteSpline(Point<Length> start_pos, Point<Length> end_pos, Point<Velocity> start_vel, Point<Velocity> end_vel) {
  a.x = 0;
  b.x = 0;
  c.x = 2 * start_pos.x() - 2 * end_pos.x() + start_vel.x() + end_vel.x();
  d.x = 3 * end_pos.x() - 3 * start_pos.x() - 2 * start_vel.x() - end_vel.x();
  e.x = start_vel.x();
  f.x = start_pos.x();

  a.y = 0;
  b.y = 0;
  c.y = 2 * start_pos.y() - 2 * end_pos.y() + start_vel.y() + end_vel.y();
  d.y = 3 * end_pos.y() - 3 * start_pos.y() - 2 * start_vel.y() - end_vel.y();
  e.y = start_vel.y();
  f.y = start_pos.y();
}

Point<Length> HermiteSpline::CalculatePosition(double p) const {
  return (a * std::pow(p, 5) +
          b * std::pow(p, 4) +
          c * std::pow(p, 3) +
          d * std::pow(p, 2) +
          e * std::pow(p, 1) +
          f * std::pow(p, 0)) * m;
}

Point<Velocity> HermiteSpline::CalculateVelocity(double p, double dp) const {
  return (5 * a * std::pow(p, 4) +
          4 * b * std::pow(p, 3) +
          3 * c * std::pow(p, 2) +
          2 * d * std::pow(p, 1) +
          e * std::pow(p, 0)) * (m / s);
}

Point<Acceleration> HermiteSpline::CalculateAcceleration(double p, double dp) const {
  return (20 * a * std::pow(p, 3) +
          12 * b * std::pow(p, 2) +
          6 * c * std::pow(p, 1) +
          2 * d * std::pow(p, 0)) * (m / s / s);
}
}
