#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>
#include "citrus_socket/network_serialize.h"

CitrusVision::CitrusVision(RobotSubsystems &subs, RobotConstants constants)
    : subsystems_(subs),
      angle_log_("angles", {"cameraAngle", "gyroHistory", "cameraProfileRunning"}),
      angle_helper_(&angle_log_),
      gyro_history_(20 * ms),
      connection_(CitrusSocket(9999)) {
  constants_ = constants;
}

Angle CitrusVision::GetAngleOff() {
  const double camera_angle = constants_.camera_scaling_factor;

  Angle camera_diff =
      (-angle_received_ + constants_.camera_offset * deg) * camera_angle;
  return camera_diff;
}

void CitrusVision::Update() {
  ReadPosition();

  has_new_image_ = (last_angle_ != angle_received_);
  last_angle_ = angle_received_;

  gyro_history_.Update(subsystems_.drive.GetGyroAngle());

  angle_log_["cameraAngle"] = std::to_string(angle_received_.to(deg));
  angle_log_["gyroHistory"] =
      std::to_string(subsystems_.drive.GetGyroAngle().to(deg));
  angle_log_["cameraProfileRunning"] = std::to_string(subsystems_.drive.IsProfileComplete());
  angle_helper_.Update();
  angle_log_.EndLine();
}

bool CitrusVision::RunVision() {
  if (!subsystems_.drive.IsProfileComplete()) {
    was_running_profile_ = true;
  } else {
    if (was_running_profile_) {
      align_timer_.Reset();
      was_running_profile_ = false;
    }

    if (align_timer_.Get() > 0.1*s) {
      if (GetAligned()) {
        aligned_for_++;
        return aligned_for_ > 4;
      } else {
        // There is usually 160 ms of lag between the image capture and being received by the robot code
        subsystems_.drive.Shift(false);
        subsystems_.drive.PointTurn(
            GetAngleOff() + (subsystems_.drive.GetGyroAngle() - gyro_history_.GoBack(160*ms)), false, false, true);
        aligned_for_ = 0;
      }
    }
  }
  return false;
}

bool CitrusVision::GetAligned() {
  return (is_found_ && has_connection_ && (muan::abs(GetAngleOff()) < 1.25 * deg));
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
