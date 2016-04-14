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
  void Update();
  bool GetAligned() { return aligned_; }
  bool IsSeeing() { return is_found_; }
  bool HasConnection() {return has_connection_; }
  bool HasNewImage() { return has_new_image_; }
  Angle GetAngleOff();
 private:
  void ReadPosition();
  void UpdateAligned();

  RobotConstants constants_;
  RobotSubsystems& subsystems_;
  CitrusSocket connection_;
  muan::CSVLog angle_log_;
  SmartDashboardHelper angle_helper_;
  bool aligned_ = false;
  int align_counter_ = 0;

  bool is_found_ = false;
  bool has_connection_ = false;
  bool has_new_image_ = false;
  bool last_align_ = false;
  Angle angle_received_ = 0*deg;
  Angle last_angle_ = 0*deg;
  Time lag_ = 0*s;
};

#endif
