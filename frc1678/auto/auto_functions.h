#ifndef AUTO_AUTO_FUNCTIONS_H_
#define AUTO_AUTO_FUNCTIONS_H_

#include "gyro/gyro_reader.h"
#include "unitscpp/unitscpp.h"
#include "muan/utils/timing_utils.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "auto_functions.h"

enum Position{
    LOW_BAR,
    BATTER,
    WORKS_3,
    WORKS_4,
};

namespace AutoFunction {
    SetUpAutoFunction();
    DeleteAutoFunction();
    bool DriveStraight(CitrusRobot* robot, Length dist, Velocity speed);
    bool DriveStraight2(CitrusRobot* robot, float dist, float speed);
    bool Turn(CitrusRobot* robot, Angle angle, Velocity speed);
    bool Wait(CitrusRobot* robot, Time time);
    bool Shoot(CitrusRobot* robot, Position infield);
    bool RunIntake(CitrusRobot* robot);
    bool DropPinch(CitrusRobot* robot);
    bool Align(CitrusRobot* robot, Angle offset);
    bool StopDriving(CitrusRobot* robot);
}

#endif

