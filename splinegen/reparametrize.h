#ifndef SPLINE_REPARAMETRIZE_H_
#define SPLINE_REPARAMETRIZE_H_

#include "reparametrized_trajectory.h"
#include "constraints.h"

namespace spline {
std::shared_ptr<ReparametrizedTrajectory> Reparametrize(std::shared_ptr<Curve> curve, DriveConstraints constraints);
}

#endif /* SPLINE_REPARAMETRIZE_H_ */
