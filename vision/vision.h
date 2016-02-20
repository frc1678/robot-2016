#ifndef _VISION_H_
#define _VISION_H_

#include "networktables/NetworkTable.h"
#include <memory>
#include "frc1678/robot_subsystems.h"
#include "muan/utils/history.h"

class CitrusVision {
 public:
  CitrusVision(RobotSubsystems& subsystems);
  void Start();
  bool Update(bool enable);
  bool IsSeeing();
  void EndTest();

 private:
  RobotSubsystems& subsystems_;
  std::shared_ptr<NetworkTable> table_;
  muan::History<Angle, 100> gyro_history_;
  muan::Timer test_timer;
  muan::CSVLog angle_log_;
};

#endif
