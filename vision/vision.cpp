#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>

CitrusVision::CitrusVision(RobotSubsystems& subs, RobotConstants constants)
    : subsystems_(subs),
      gyro_history_(.02 * s),
      angle_log_("angles", {"cameraAngle", "gyroHistory"}) {
  table_ = NetworkTable::GetTable("vision");
  table_->PutBoolean("targetFound", false);
  constants_ = constants;
}

void CitrusVision::Start() {
  const Angle camera_angle = 1.136 * deg;

  float raw_table_angle = -table_->GetNumber("angleToTarget", 0);
  Angle camera_diff = (-table_->GetNumber("angleToTarget", 0) + constants_.camera_offset) * camera_angle;
  printf("cam: got %f from camera, turning %f\n", raw_table_angle, camera_diff.to(deg));
  subsystems_.drive.PointTurn(camera_diff, false);
}

bool CitrusVision::IsSeeing() {
  return table_->GetBoolean("targetFound", false);
}

bool CitrusVision::Update(bool enabled) {
  return subsystems_.drive.IsProfileComplete();
}

void CitrusVision::EndTest() {
  // test_log_["end"] = std::to_string(table_->GetNumber("angleToTarget", 0));
  // test_log_.EndTest();
}
