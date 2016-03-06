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

    ret.camera_offset = -7.5;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .6;
  } else if (id == RobotIdentifier::APPA) {
    ret.pivot_calibration_offset = 22.8 * deg;

    ret.pivot_gains = {80 * V / rad, 0 * V / (rad * s), 1 * V / (rad / s)};
    ret.pivot_climb_gains = {140 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = .5;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .85;
  } else if (id == RobotIdentifier::COMP) {
    ret.pivot_calibration_offset = 21.6 * deg;

    ret.pivot_gains = {50 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -0;  // TODO(Wesley) convert to unit? Also get real
                             // value
    ret.pivot_efficiency = .85;
  }
  return ret;
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
