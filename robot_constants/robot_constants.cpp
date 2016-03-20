#include "robot_constants.h"
#include "robot_identifier.h"
#include "muan/unitscpp/unitscpp.h"

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::SSBB) {
    ret.pivot_calibration_offset = 21.2 * deg;

    ret.pivot_gains = {140 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -2.2;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .6;
  } else if (id == RobotIdentifier::APPA) {
    ret.pivot_calibration_offset = 24.8 * deg;

    ret.pivot_gains = {80 * V / rad, 0 * V / (rad * s), 1 * V / (rad / s)};
    ret.pivot_climb_gains = {140 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -4.1;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .85;
  } else if (id == RobotIdentifier::COMP) {
    ret.pivot_calibration_offset = 20.4 * deg;

    ret.pivot_gains = {80 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {100 * V / m, 20 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -0.8;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .85;
  }
  return ret;
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
