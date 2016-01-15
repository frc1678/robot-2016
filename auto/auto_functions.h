#ifndef AUTO_AUTO_FUNCTIONS_H_
#define AUTO_AUTO_FUNCTIONS_H_

#include "gyro/gyro_reader.h"
#include "unitscpp/unitscpp.h"
#include "muan/utils/timing_utils.h"
#include <WPILib.h>
#include "auto_functions.h"

namespace AutoFunction {
    void Drivestraight(Length dist, Velocity speed);
    void Turn(Angle angle, Velocity speed);
    void Wait(Time time);
    void Shoot(Length distance);
    void Intake();
    void DropPinch();
    void Align();
    void Stop();
} 

#endif
