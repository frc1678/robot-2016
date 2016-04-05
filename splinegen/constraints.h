#ifndef SPLINE_CONSTRAINTS_H_
#define SPLINE_CONSTRAINTS_H_

#include "muan/unitscpp/unitscpp.h"

class DriveConstraints {
 public:
  DriveConstraints(Velocity max_forward_velocity, Acceleration max_forward_acceleration,
                   AngularVelocity max_angular_velocity, AngularAcceleration max_angular_acceleration);

  Time SegmentTravelTime(Length seg_length, Angle seg_angle, Velocity initial_forward_velocity, AngularVelocity initial_angular_velocity);
 private:
  Velocity max_forward_velocity_;
  Acceleration max_forward_acceleration_;
  AngularVelocity max_angular_velocity_;
  AngularAcceleration max_angular_acceleration_;
};

#endif /* end of include guard: SPLINE_CONSTRAINTS_H_ */
