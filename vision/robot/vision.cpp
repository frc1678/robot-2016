#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>
#include "citrus_socket/network_serialize.h"

CitrusVision::CitrusVision(RobotSubsystems &subs, RobotConstants constants)
    : subsystems_(subs),
      angle_log_("angles", {"cameraAngle", "gyroHistory", "cameraProfileRunning"}),
      angle_helper_(&angle_log_),
      connection_(CitrusSocket(1678)) {
  constants_ = constants;
}

Angle CitrusVision::GetAngleOff() {
  const double camera_angle = constants_.camera_scaling_factor;

  Angle camera_diff =
      (-angle_received_ + constants_.camera_offset * deg) * camera_angle;
  return camera_diff;
}

void CitrusVision::Start() {
  subsystems_.drive.PointTurn(GetAngleOff(), false);
  align_counter_ = 0;
}

void CitrusVision::Update() {
  ReadPosition();
  UpdateAligned();

  has_new_image_ = (last_angle_ != angle_received_);
  last_angle_ = angle_received_;

  angle_log_["cameraAngle"] = std::to_string(angle_received_.to(deg));
  angle_log_["gyroHistory"] =
      std::to_string(subsystems_.drive.GetGyroAngle().to(deg));
  angle_log_["cameraProfileRunning"] = std::to_string(subsystems_.drive.IsProfileComplete());
  angle_helper_.Update();
  angle_log_.EndLine();
}

//TODO(Wesley) make this work at any frequency
void CitrusVision::UpdateAligned() {
  Angle tolerance = 1.25 * deg;
  if (is_found_ && has_connection_ && (muan::abs(GetAngleOff()) < tolerance) && last_align_) {
    align_counter_++;
    last_align_ = true;
  } else if (is_found_ && has_connection_ && (muan::abs(GetAngleOff()) < tolerance) && !last_align_) {
    align_counter_ = 0;
    last_align_ = true;
  } else {
    last_align_ = false;
    align_counter_ = 0;
  }

  aligned_ = align_counter_ > 10;
}

bool CitrusVision::GetAligned() {
  return aligned_;
}

void CitrusVision::ReadPosition() {
  bool got_packets = false;
  try {
    nlohmann::json json_recv;
    while (connection_.HasPackets()) {
      json_recv = to_json(connection_.Receive());
    }
    is_found_ = json_recv["found"];
    angle_received_ = json_recv["angle"] * deg;
    lag_ = json_recv["lag"] * s;
    got_packets = true;
  } catch (...) {
  }
  has_connection_ = got_packets;
}
