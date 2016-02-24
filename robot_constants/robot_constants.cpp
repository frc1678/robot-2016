#include "robot_constants.h"
#include "robot_identifier.h"

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::PRACTICE_ROBOT_1) {
    ret.pivot_calibration_offset = 20.2 * deg;

    ret.pivot_gains = {40 * V / rad, 15 * V / (rad * s), 1 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};
  } else if (id == RobotIdentifier::PRACTICE_ROBOT_2) {
    ret.pivot_calibration_offset = 23.4 * deg;

    ret.pivot_gains = {40 * V / rad, 0 * V / (rad * s), 1 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};
  } else if (id == RobotIdentifier::COMPETITION_ROBOT) {
    ret.pivot_calibration_offset = 21.6 * deg;

    ret.pivot_gains = {40 * V / rad, 0 * V / (rad * s), 1 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};
  }
  return ret;
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
