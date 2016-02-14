#ifndef AUTO_AUTO_FUNCTIONS_H_
#define AUTO_AUTO_FUNCTIONS_H_

#include "gyro/gyro_reader.h"
#include "unitscpp/unitscpp.h"
#include "muan/utils/timing_utils.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "auto_functions.h"
#include "frc1678/frc1678.h"

enum Position { 
  LONG = 0,
  TUCK,
  INTAKE
};

namespace AutoFunction {
void SetUpAutoFunction();
void DeleteAutoFunction();
bool DriveStraight(CitrusRobot* robot, float dist); //TODO(Wesley) Change name to be the same as elsewhere
bool DriveStraightAtAngle(CitrusRobot* robot, float dist, float angle);
bool PointTurn(CitrusRobot *robot, float angle);
bool AbsolutePointTurn(CitrusRobot *robot, float angle);
bool Wait(CitrusRobot* robot, float time);
bool Shoot(CitrusRobot* robot);
bool RunIntake(CitrusRobot* robot);
bool SetArmPosition(CitrusRobot* robot, Position arm_position);
bool DropPinch(CitrusRobot* robot);
bool Align(CitrusRobot* robot);
bool StopDriving(CitrusRobot* robot);
}

#endif
