#ifndef ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_
#define ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_

#include "unitscpp/unitscpp.h"

class RobotConstants {
 public:
  static const RobotConstants& GetInstance();

  Angle pivot_calibration_offset;

 private:
  static RobotConstants instance;
};

#endif /* ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_ */
