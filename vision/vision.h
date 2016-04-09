#ifndef _VISION_H_
#define _VISION_H_

#include "networktables/NetworkTable.h"
#include <memory>
#include "frc1678/robot_subsystems.h"
#include "muan/utils/history.h"
#include "citrus_socket/citrus_socket.h"

class CitrusVision {
 public:
  CitrusVision(RobotSubsystems& subsystems, RobotConstants constants);
  void Start();
  bool Update(bool enable);
  void EndTest();
  bool Aligned();
  bool InitallyAligned() { return (!Aligned() && align_counter_ >= 1); }
  bool IsSeeing() { return isFound; }
  bool HasConnection() {return hasConnection; }
  bool HasNewImage() { return hasNewImage; }
  int align_counter_ = 0;
 private:
  void ReadPosition();
  Angle GetAngleOff();

  RobotConstants constants_;
  RobotSubsystems& subsystems_;
  CitrusSocket connection;
  muan::History<Angle, 100> gyro_history_;
  muan::Timer test_timer;
  muan::CSVLog angle_log_;
  SmartDashboardHelper angle_helper_;

  bool isFound, hasConnection, hasNewImage;
  bool last_align_ = false;
  Angle angleReceived;
  Angle last_angle_;
  Time lag;
};

#endif
