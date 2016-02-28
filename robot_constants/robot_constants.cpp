#include "robot_constants.h"
#include "robot_identifier.h"

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::SSBB) {
    ret.pivot_calibration_offset = 20.2 * deg;

    ret.pivot_gains = {10 * V / rad, 0 * V / (rad * s), 0 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.pivot_efficiency = .85;
  } else if (id == RobotIdentifier::APPA) {
    ret.pivot_calibration_offset = 23.4 * deg;

    ret.pivot_gains = {40 * V / rad, 0 * V / (rad * s), 0 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.pivot_efficiency = .85;
  } else if (id == RobotIdentifier::COMP) {
    ret.pivot_calibration_offset = 21.6 * deg;

    ret.pivot_gains = {50 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};
  }
  return ret;
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
