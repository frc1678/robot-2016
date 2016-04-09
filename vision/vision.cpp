#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>
#include "citrus_socket/network_serialize.h"

CitrusVision::CitrusVision(RobotSubsystems &subs, RobotConstants constants)
    : subsystems_(subs),
      gyro_history_(.02 * s),
      angle_log_("angles", {"cameraAngle", "gyroHistory", "cameraProfileRunning"}),
      angle_helper_(&angle_log_),
      connection(CitrusSocket(1678)) {
  constants_ = constants;
  isFound = false;
  hasConnection = false;
  angleReceived = 0 * deg;
  last_angle_ = 0 * deg; lag = 0 * s;
}

Angle CitrusVision::GetAngleOff() {
  const double camera_angle = constants_.camera_scaling_factor;
// Axis camera      1.136;  // multiply by this to turn camera angles into gyro angles

  Angle camera_diff =
      (-angleReceived + constants_.camera_offset * deg) * camera_angle;
  return camera_diff;
}

void CitrusVision::Start() {
  subsystems_.drive.PointTurn(GetAngleOff(), false);
  align_counter_ = 0;
}

bool CitrusVision::Update(bool enabled) {
  ReadPosition();
  angle_log_["cameraAngle"] = std::to_string(angleReceived.to(deg));
  angle_log_["gyroHistory"] =
      std::to_string(subsystems_.drive.GetGyroAngle().to(deg));
  angle_log_["cameraProfileRunning"] = std::to_string(subsystems_.drive.IsProfileComplete());
  angle_helper_.Update();
  angle_log_.EndLine();
  if( last_angle_ == angleReceived) {
    hasNewImage = false;
  } else { hasNewImage = true; }
  last_angle_ = angleReceived;
  return subsystems_.drive.IsProfileComplete();
}

bool CitrusVision::Aligned() {
  Angle tolerance = 1.25 * deg;
  if (isFound && hasConnection && (muan::abs(GetAngleOff()) < tolerance) && last_align_) {
    align_counter_++;
    last_align_ = true;
  } else if (isFound && hasConnection && (muan::abs(GetAngleOff()) < tolerance) && !last_align_) {
    align_counter_ = 0;
    last_align_ = true;
  } else {
    last_align_ = false;
    align_counter_ = 0;
  }
  
          
  if (align_counter_ > 10){
    return true;
  } else {
    return false;
  }
}

void CitrusVision::EndTest() {
  // test_log_["end"] = std::to_string(table_->GetNumber("angleToTarget", 0));
  // test_log_.EndTest();
}

void CitrusVision::ReadPosition() {
  bool gotPackets = false;
  try {
    nlohmann::json json_recv;
    while (connection.HasPackets()) {
      json_recv = to_json(connection.Receive());
    }
    isFound = json_recv["found"];
    angleReceived = json_recv["angle"] * deg;
    lag = json_recv["lag"] * s;
    gotPackets = true;
  } catch (...) {
  }
  hasConnection = gotPackets;
}
