#ifndef ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_
#define ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_

#include "muan/unitscpp/unitscpp.h"
#include "muan/control/pid_controller.h"

struct ArmGoal {
  Angle pivot_goal;
  Length elevator_goal;
  AngularVelocity shooter_goal;
};

class RobotConstants {
 public:
  static const RobotConstants& GetInstance();
  static void ReloadConstants();

  Angle pivot_calibration_offset;

  muan::PidController<Angle, Voltage>::PidGains pivot_gains;
  muan::PidController<Angle, Voltage>::PidGains pivot_climb_gains;

  muan::PidController<Angle, Voltage>::PidGains drivetrain_angle_gains;
  muan::PidController<Length, Voltage>::PidGains drivetrain_distance_gains;
  muan::PidController<Angle, Voltage>::PidGains vision_angle_gains;

  muan::PidController<Length, Voltage>::PidGains elevator_gains;

  float camera_offset;
  double pivot_efficiency;
  float camera_scaling_factor;

  ArmGoal long_shot_goals, auto_shot_goals, fender_shot_goals;

 private:
  static RobotConstants instance;
};

#endif /* ROBOT_CONSTANTS_ROBOT_CONSTANTS_H_ */
