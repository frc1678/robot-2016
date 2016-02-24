#ifndef ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_
#define ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_

#include "unitscpp/unitscpp.h"
#include "muan/control/pid_controller.h"

class RobotConstants {
 public:
  static const RobotConstants& GetInstance();

  Angle pivot_calibration_offset;

  muan::PidController<Angle, Voltage>::PidGains pivot_gains;
  muan::PidController<Angle, Voltage>::PidGains pivot_climb_gains;

  muan::PidController<Length, Voltage>::PidGains elevator_gains;

 private:
  static RobotConstants instance;
};

#endif /* ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_ */
