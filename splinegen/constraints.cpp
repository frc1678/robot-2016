#include "constraints.h"
#include "muan/utils/math_utils.h"
#include <cmath>
#include <algorithm>

DriveConstraints::DriveConstraints(Velocity max_forward_velocity, Acceleration max_forward_acceleration,  AngularVelocity max_angular_velocity, AngularAcceleration max_angular_acceleration) : max_forward_velocity_(max_forward_velocity), max_forward_acceleration_(max_forward_acceleration), max_angular_velocity_(max_angular_velocity), max_angular_acceleration_(max_angular_acceleration) {}

double solve_quadratic(double a, double b, double c) {
  return (-b + std::sqrt(b*b - 4*a*c)) / (2*a);
}

Time DriveConstraints::SegmentTravelTime(Length seg_length, Angle seg_angle, Velocity initial_forward_velocity, AngularVelocity initial_angular_velocity) {
  Time time_vel = muan::abs(seg_length / max_forward_velocity_) + muan::abs(seg_angle / max_angular_velocity_);

  Time time_forward_acceleration = solve_quadratic(max_forward_acceleration_(),
                                                   muan::abs(initial_forward_velocity()),
                                                   -muan::abs(seg_length())) * s;

  Time time_angular_acceleration = solve_quadratic(max_angular_acceleration_(),
                                                   muan::abs(initial_angular_velocity()),
                                                   -muan::abs(seg_angle())) * s;

  return std::max({time_vel, time_forward_acceleration, time_angular_acceleration});
}
