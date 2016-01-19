#ifndef _VISION_H_
#define _VISION_H_

#include "drivetrain/drivetrain_subsystem.h"
#include "networktables/NetworkTable.h"

namespace CitrusVision {

void start(DrivetrainSubsystem* ds);
void updateVision(DrivetrainSubsystem* ds);

}

#endif
