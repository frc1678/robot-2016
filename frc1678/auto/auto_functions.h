#ifndef AUTO_AUTO_FUNCTIONS_H_
#define AUTO_AUTO_FUNCTIONS_H_

#include "gyro/gyro_reader.h"
#include "unitscpp/unitscpp.h"
#include "muan/utils/timing_utils.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "auto_functions.h"
#include "frc1678/robot_subsystems.h"

enum Position {
  LOW_BAR,
  BATTER,
  WORKS_3,
  WORKS_4,
};

namespace AutoFunction {
void SetUpAutoFunction();
void DeleteAutoFunction();
bool DriveStraight(RobotSubsystems* subs, Length dist, Velocity speed);
bool DriveStraight2(RobotSubsystems* subs, float dist, float speed);
bool Turn(RobotSubsystems* subs, Angle angle, Velocity speed);
bool Wait(RobotSubsystems* subs, Time time);
bool Shoot(RobotSubsystems* subs, Position infield);
bool RunIntake(RobotSubsystems* subs);
bool DropPinch(RobotSubsystems* subs);
bool Align(RobotSubsystems* subs, Angle offset);
bool StopDriving(RobotSubsystems* subs);
}

#endif
