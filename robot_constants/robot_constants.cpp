#include "robot_constants.h"
#include "robot_identifier.h"
#include "muan/unitscpp/unitscpp.h"
#include <iostream>
#include <fstream>
#include <string>

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

template<typename I, typename T>
typename muan::PidController<I,T>::PidGains LoadConstantsFromFile(std::string filename) {
  double p_constant = 0;
  double i_constant = 0;
  double d_constant = 0;
  std::ifstream constants_file("/home/lvuser/constants/" + filename);
  if (constants_file.is_open())
  {
    constants_file >> p_constant >> i_constant >> d_constant;
    constants_file.close();
  }
  return typename muan::PidController<I, T>::PidGains {p_constant * T(1)/I(1), i_constant * T(1)/(I(1)*s), d_constant * T(1)/(I(1)/s)};
}

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::SSBB) {
    ret.pivot_calibration_offset = 21.2 * deg;
    ret.pivot_efficiency = .6;
    ret.camera_offset = -7.5;
  } else if (id == RobotIdentifier::APPA) {
    ret.pivot_calibration_offset = 24.8 * deg;
    ret.pivot_efficiency = .85;
    ret.camera_offset = -4.1;
  } else if (id == RobotIdentifier::COMP) {
    ret.pivot_calibration_offset = 20.4 * deg;
    ret.pivot_efficiency = .85;
    ret.camera_offset = -0.8;
  }
  ret.pivot_gains = LoadConstantsFromFile<Angle, Voltage>(GetRobotString(id) + "/pivot_gains");
  ret.pivot_climb_gains = LoadConstantsFromFile<Angle, Voltage>(GetRobotString(id) + "/pivot_climb_gains");
  ret.elevator_gains = LoadConstantsFromFile<Length, Voltage>(GetRobotString(id) + "/elevator_gains");
  ret.drivetrain_angle_gains = LoadConstantsFromFile<Angle, Voltage>(GetRobotString(id) + "/drivetrain_angle_gains");
  ret.drivetrain_distance_gains = LoadConstantsFromFile<Length, Voltage>(GetRobotString(id) + "/drivetrain_distance_gains");
  return ret;
}

void RobotConstants::ReloadConstants() {
  RobotConstants::instance = GenerateRobotConstants(GetRobotIdentifier());
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
