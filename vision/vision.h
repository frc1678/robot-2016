#ifndef _VISION_H_
#define _VISION_H_

#include "networktables/NetworkTable.h"
#include <memory>
#include "frc1678/robot_subsystems.h"
#include "muan/control/trapezoidal_motion_profile.h"

class CitrusVision {
 public:
  CitrusVision(RobotSubsystems& subsystems);
  void Start();
  bool Update();
  void EndTest();

 private:
  Angle start_vision_angle_;
  Angle start_gyro_angle_;
  RobotSubsystems& subsystems_;
  std::shared_ptr<NetworkTable> table_;
  muan::PidController<Angle, Voltage> turn_controller_;
};

#endif
