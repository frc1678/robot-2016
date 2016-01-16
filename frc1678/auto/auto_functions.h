#ifndef AUTO_AUTO_FUNCTIONS_H_
#define AUTO_AUTO_FUNCTIONS_H_

#include "gyro/gyro_reader.h"
#include "unitscpp/unitscpp.h"
#include "muan/utils/timing_utils.h"
#include "auto_functions.h"

enum Position{
    LOW_BAR,
    BATTER,
    WORKS_3,
    WORKS_4,
};

class AutoFunction {
  public:
    AutoFunction();
    ~AutoFunction();
    bool Drivestraight(Length dist, Velocity speed);
    bool Turn(Angle angle, Velocity speed);
    bool Wait(Time time);
    bool Shoot(Position infield);
    bool RunIntake();
    bool DropPinch();
    bool Align(Angle offset);
    bool StopDriving();
};

#endif
