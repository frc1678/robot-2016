#include "robot_constants.h"
#include "robot_identifier.h"
#include "muan/unitscpp/unitscpp.h"
#include <iostream>

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::SSBB) {
    std::cout << "Robot Id: SSBB" << std::endl;
    ret.pivot_calibration_offset = 21.2 * deg;

    ret.pivot_gains = {140 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -7.5;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .6;

    ret.long_shot_goals = {42 * deg, .33 * m, 6500 * rev / (60 * s)};
    ret.auto_shot_goals = {36 * deg, 0 * m, 5500 * rev / (60 * s)};
    ret.fender_shot_goals = {10 * deg, 0 * m, 5500 * rev / (60 * s)};
  } else if (id == RobotIdentifier::APPA) {
    std::cout << "Robot Id: APPA" << std::endl;
    ret.pivot_calibration_offset = 24.8 * deg;

    ret.pivot_gains = {80 * V / rad, 0 * V / (rad * s), 1 * V / (rad / s)};
    ret.pivot_climb_gains = {140 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -4.1;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .85;

    ret.long_shot_goals = {42 * deg, .33 * m, 6500 * rev / (60 * s)};
    ret.auto_shot_goals = {36 * deg, 0 * m, 5500 * rev / (60 * s)};
    ret.fender_shot_goals = {10 * deg, 0 * m, 5500 * rev / (60 * s)};
  } else if (id == RobotIdentifier::COMP) {
    std::cout << "Robot Id: COMP" << std::endl;
    ret.pivot_calibration_offset = 20.4 * deg;

    ret.pivot_gains = {80 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {100 * V / m, 20 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -0.8;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .85;

    ret.long_shot_goals = {42 * deg, .33 * m, 6500 * rev / (60 * s)};
    ret.auto_shot_goals = {36 * deg, 0 * m, 5500 * rev / (60 * s)};
    ret.fender_shot_goals = {10 * deg, 0 * m, 5500 * rev / (60 * s)};
  }
  return ret;
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
