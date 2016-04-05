#ifndef SPLINE_REPARAMETRIZED_TRAJECTORY_H_
#define SPLINE_REPARAMETRIZED_TRAJECTORY_H_

#include "trajectory.h"
#include <vector>
#include <memory>
#include "curve.h"
#include "muan/unitscpp/unitscpp.h"

namespace spline {

class ReparametrizedTrajectory : public Trajectory {
 public:
  ReparametrizedTrajectory(std::shared_ptr<Curve> curve, std::vector<Time> t_values, double p_step);
  Time GetTotalTime() override;

  Length CalculateArcLength(Time t) override;
  Velocity CalculateForwardVelocity(Time t) override;
  Acceleration CalculateForwardAcceleration(Time t) override;

  Angle CalculateHeading(Time t) override;
  AngularVelocity CalculateAngularVelocity(Time t) override;
  AngularAcceleration CalculateAngularAcceleration(Time t) override;

 private:
  double CalculateSValue(Time t);
  double PPrime(Time t) {
    CalculateSValue(t);
    if (last_index_ == 0) return 0;
    return p_step_ / (t_values_[last_index_] - t_values_[last_index_ - 1])();
  }

  std::shared_ptr<Curve> curve_;
  std::vector<Time> t_values_;
  double p_step_;
  int last_index_;
};
}

#endif /* end of include guard: SPLINE_REPARAMETRIZED_TRAJECTORY_H_ */
