#ifndef FRC1678_ROBOT_SUBSYSTEMS_H_
#define FRC1678_ROBOT_SUBSYSTEMS_H_

#include "drivetrain/drivetrain_subsystem.h"
#include "arm/arm_subsystem.h"

struct RobotSubsystems {
  DrivetrainSubsystem drive;
  ArmSubsystem arm;
};

#endif /* FRC1678_ROBOT_SUBSYSTEMS_H_ */
