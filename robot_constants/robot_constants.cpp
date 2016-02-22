#include "robot_constants.h"
#include "robot_identifier.h"

const RobotConstants& RobotConstants::GetInstance() {
  return RobotConstants::instance;
}

RobotConstants GenerateRobotConstants(RobotIdentifier id) {
  RobotConstants ret;
  if (id == RobotIdentifier::PRACTICE_ROBOT_1) {
    ret.pivot_calibration_offset = 20.2 * deg;
  } else if (id == RobotIdentifier::PRACTICE_ROBOT_2) {
    ret.pivot_calibration_offset = 23.4 * deg;
  } else if (id == RobotIdentifier::COMPETITION_ROBOT) {
    ret.pivot_calibration_offset = 21.6 * deg;
  }
  return ret;
}

RobotConstants RobotConstants::instance =
    GenerateRobotConstants(GetRobotIdentifier());
