#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>

CitrusVision::CitrusVision(RobotSubsystems& subs)
    : subsystems_(subs),
      turn_controller_(10 * V / rad, 0 * V / (rad * s), 0 * V / (rad / s)),
      gyro_history_(.02 * s) {
  table_ = NetworkTable::GetTable("vision");
}

void CitrusVision::Start() {
  Angle angle = -table_->GetNumber("angleToTarget", 0) * deg;
  using muan::TrapezoidalMotionProfile;
  auto distance_profile = std::make_unique<TrapezoidalMotionProfile<Length>>(
      0 * m, 10 * m / s, 10 * m / s / s);
  auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
      angle, 3.5 * rad / s, 270 * deg / s / s);
  subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
                                        std::move(angle_profile));
}

bool CitrusVision::Update() {
  Angle camera_angle_diff = -table_->GetNumber("angleToTarget", 0) * deg;
  // Where the robot was at the time the image was taken
  Angle gyro_angle = gyro_history_.GoBack(table_->GetNumber("latency", 0) * s);
  Angle target_angle = gyro_angle + camera_angle_diff;
  Voltage v = turn_controller_.Calculate(
      .02 * s, target_angle - subsystems_.drive.GetGyroAngle());
  DrivetrainGoal goal;
  goal.quickturn = true;
  goal.throttle = false;
  goal.highgear = false;
  goal.steering = v.to(12 * V) / 4.0;
  subsystems_.drive.SetDriveGoal(goal);
  gyro_history_.Update(subsystems_.drive.GetGyroAngle());
  return false;
}

void CitrusVision::EndTest() {
  // test_log_["end"] = std::to_string(table_->GetNumber("angleToTarget", 0));
  // test_log_.EndTest();
}
