#include "robot_constants.h"
#include "robot_identifier.h"
#include "muan/unitscpp/unitscpp.h"
#include <iostream>
#include <fstream>
#include <string>

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

template <typename I, typename T>
typename muan::PidController<I, T>::PidGains LoadConstantsFromFile(
    std::string filename) {
  double p_constant = 0;
  double i_constant = 0;
  double d_constant = 0;
  std::ifstream constants_file("/home/lvuser/constants/" + filename);
  if (constants_file.is_open()) {
    constants_file >> p_constant >> i_constant >> d_constant;
    constants_file.close();
  }
  return typename muan::PidController<I, T>::PidGains{
      p_constant * T(1) / I(1), i_constant * T(1) / (I(1) * s),
      d_constant * T(1) / (I(1) / s)};
}

// flip camera offset from what you want it to display as

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::SSBB) {
    ret.pivot_calibration_offset = 21.2 * deg;

    ret.pivot_gains = {140 * V / rad, 0 * V / (rad * s), 2 * V / (rad / s)};
    ret.pivot_climb_gains = {100 * V / rad, 40 * V / (rad * s),
                             0 * V / (rad / s)};

    ret.elevator_gains = {60 * V / m, 10 * V / (m * s), 0 * V / (m / s)};

    ret.camera_offset = -0.9;  // TODO(Wesley) convert to unit?
    ret.pivot_efficiency = .6;
    ret.camera_scaling_factor = 1.01;

    ret.long_shot_goals = {42 * deg, .33 * m, 6500 * rev / (60 * s)};
    ret.auto_shot_goals = {41.5 * deg, 0 * m, 5500 * rev / (60 * s)};
    ret.fender_shot_goals = {10 * deg, 0 * m, 5500 * rev / (60 * s)};
  } else if (id == RobotIdentifier::APPA) {
    ret.pivot_calibration_offset = 24.0 * deg;
    ret.pivot_efficiency = .85;
    // Right is negative
    ret.camera_offset = -2;
    ret.camera_scaling_factor = 1.1;

    ret.long_shot_goals = {42 * deg, .33 * m, 6500 * rev / (60 * s)};
    ret.auto_shot_goals = {34 * deg, 0 * m, 7000 * rev / (60 * s)};
    ret.fender_shot_goals = {10 * deg, 0 * m, 5500 * rev / (60 * s)};
  } else if (id == RobotIdentifier::COMP) {
    ret.pivot_calibration_offset = 20.4 * deg;
    ret.pivot_efficiency = .85;
    ret.camera_offset = -.514;
    ret.camera_scaling_factor = 1.1;

    ret.long_shot_goals = {42 * deg, .33 * m, 7000 * rev / (60 * s)};
    ret.auto_shot_goals = {36 * deg, 0 * m, 6500 * rev / (60 * s)};
    ret.fender_shot_goals = {10 * deg, 0 * m, 6500 * rev / (60 * s)};
  }
  ret.pivot_gains = LoadConstantsFromFile<Angle, Voltage>(GetRobotString(id) +
                                                          "/pivot_gains");
  ret.pivot_climb_gains = LoadConstantsFromFile<Angle, Voltage>(
      GetRobotString(id) + "/pivot_climb_gains");
  ret.elevator_gains = LoadConstantsFromFile<Length, Voltage>(
      GetRobotString(id) + "/elevator_gains");
  ret.drivetrain_angle_gains = LoadConstantsFromFile<Angle, Voltage>(
      GetRobotString(id) + "/drivetrain_angle_gains");
  ret.vision_angle_gains = LoadConstantsFromFile<Angle, Voltage>(
      GetRobotString(id) + "/vision_angle_gains");
  ret.drivetrain_distance_gains = LoadConstantsFromFile<Length, Voltage>(
      GetRobotString(id) + "/drivetrain_distance_gains");
  return ret;
}

void RobotConstants::ReloadConstants() {
  RobotConstants::instance = GenerateRobotConstants(GetRobotIdentifier());
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());



    // 1.236 is the actual scaling factor for APPA and is 1.34 for SSBB
