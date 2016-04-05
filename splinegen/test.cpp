#include "splinegen/hermite_spline.h"
#include "splinegen/constraints.h"
#include "splinegen/reparametrize.h"
#include <fstream>
#include <iostream>

int main() {
  spline::Point<Length> start(0 * m, 0 * m);
  spline::Point<Length> end(1 * m, 1 * m);
  spline::Point<Velocity> start_vel(1 * m / s, 0 * m / s);
  spline::Point<Velocity> end_vel(1 * m / s, 0 * m / s);
  auto spline = std::make_shared<spline::HermiteSpline>(start, end, start_vel, end_vel);
  auto reparametrized = spline::Reparametrize(spline, DriveConstraints(1.44, 380 * deg / s, 10 * ft / s / s, 500 * deg / s / s));
  std::ofstream file("test.csv");
  std::cout << reparametrized->GetTotalTime() << std::endl;
  for (Time t = 0 * s; t < reparametrized->GetTotalTime(); t += .01 * s) {
    file << t << ", " << reparametrized->CalculateForwardVelocity(t) << std::endl;
  }
  file.close();
}
