#include "reparametrize.h"

namespace spline {
std::shared_ptr<ReparametrizedTrajectory> Reparametrize(std::shared_ptr<Curve> curve, DriveConstraints constraints) {
  std::vector<Time> dt_values_;
  Velocity lfv = 0 * m / s;
  AngularVelocity lav = 0 * rad / s;
  double dp = 0.001;
  for (int i = 0; i < static_cast<int>(1.0 / dp); i++) {
    double p = i * dp;

    Time dt = constraints.SegmentTravelTime(curve->CalculateForwardVelocity(p) * dp * s,
                                            curve->CalculateAngularVelocity(p) * dp * s,
                                            lfv, lav);

    lfv = dp / dt() * curve->CalculateForwardVelocity(p);
    lav = dp / dt() * curve->CalculateAngularVelocity(p);

    dt_values_.push_back(dt);
  }

  lfv = 0 * m / s;
  lav = 0 * rad / s;

  for (int i = static_cast<int>(1.0 / dp) - 1; i >= 0; i--) {
    double p = i * dp;

    Time dt = constraints.SegmentTravelTime(curve->CalculateForwardVelocity(p) * dp * s,
                                            curve->CalculateAngularVelocity(p) * dp * s,
                                            lfv, lav);

    lfv = dp / dt() * curve->CalculateForwardVelocity(p);
    lav = dp / dt() * curve->CalculateAngularVelocity(p);

    dt_values_[i] = std::max(dt, dt_values_[i]);
  }

  for (int i = 1; i < dt_values_.size(); i++) {
    dt_values_[i] += dt_values_[i-1];
  }

  return std::make_shared<ReparametrizedTrajectory>(curve, dt_values_, dp);
}
}
