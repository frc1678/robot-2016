#include "reparametrized_trajectory.h"

namespace spline {

ReparametrizedTrajectory::ReparametrizedTrajectory(std::shared_ptr<Curve> curve, std::vector<Time> t_values, double p_step) : curve_(curve), t_values_(t_values) {
  p_step_ = p_step;
  last_index_ = 0;
}

Time ReparametrizedTrajectory::GetTotalTime() {
  return t_values_.back();
}

Length ReparametrizedTrajectory::CalculateArcLength(Time t) {
  return curve_->CalculateArcLength(0, CalculateSValue(t));
}

Velocity ReparametrizedTrajectory::CalculateForwardVelocity(Time t) {
  return curve_->CalculateForwardVelocity(CalculateSValue(t)) * PPrime(t);
}

Acceleration ReparametrizedTrajectory::CalculateForwardAcceleration(Time t) {
  return curve_->CalculateForwardAcceleration(CalculateSValue(t)) * PPrime(t);
}

Angle ReparametrizedTrajectory::CalculateHeading(Time t) {
  return curve_->CalculateHeading(CalculateSValue(t));
}

AngularVelocity ReparametrizedTrajectory::CalculateAngularVelocity(Time t) {
  return curve_->CalculateAngularVelocity(CalculateSValue(t)) * PPrime(t);
}

AngularAcceleration ReparametrizedTrajectory::CalculateAngularAcceleration(Time t) {
  return curve_->CalculateAngularAcceleration(CalculateSValue(t)) * PPrime(t);
}

double ReparametrizedTrajectory::CalculateSValue(Time t) {
  for (; last_index_ < t_values_.size() && t > t_values_[last_index_]; last_index_++) {}
  double p = last_index_ * p_step_;

  return p - p_step_ + ((t - t_values_[last_index_ - 1]) / (t_values_[last_index_] - t_values_[last_index_ - 1]) * p_step_)();
}

}
