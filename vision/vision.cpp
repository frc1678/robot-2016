#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>

CitrusVision::CitrusVision(RobotSubsystems& subs) : subsystems_(subs), gyro_history_(.02 * s) {
  table_ = NetworkTable::GetTable("vision");
}

void CitrusVision::Start() {
  /*    Angle angle = -table_->GetNumber("angleToTarget", 0) * deg;
      using muan::TrapezoidalMotionProfile;
      auto distance_profile =
          std::make_unique<TrapezoidalMotionProfile<Length>>(0*m, 10*m/s,
     10*m/s/s);
      auto angle_profile =
     std::make_unique<TrapezoidalMotionProfile<Angle>>(angle, 3.5*rad/s,
     270*deg/s/s);
      subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
                                            std::move(angle_profile));*/
  // Angle angle = -table_->GetNumber("angleToTarget", 0) * deg;
  // using muan::TrapezoidalMotionProfile;
  // auto distance_profile = std::make_unique<TrapezoidalMotionProfile<Length>>(
  //     0 * m, 10 * m / s, 10 * m / s / s);
  // auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
  //     angle, 3.5 * rad / s, 270 * deg / s / s);
  // subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
  //                                       std::move(angle_profile));
}

bool CitrusVision::Update() {
  Angle camera_diff = -table_->GetNumber("angleToTarget", 0) * deg;
  Time latency = table_->GetNumber("captureTime", -100) * s;
  // Angle camera_diff = -SmartDashboard::GetNumber("angleToTarget", 0) * deg;
  // Time latency = SmartDashboard::GetNumber("captureTime", -100);
  Angle target_angle = camera_diff + gyro_history_.GoBack(latency);
  if (subsystems_.drive.IsProfileComplete() && latency >= 0*s) {
    using muan::TrapezoidalMotionProfile;
    auto distance_profile = std::make_unique<TrapezoidalMotionProfile<Length>>(
        0 * m, 10 * m / s, 10 * m / s / s);
    auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
        target_angle - subsystems_.drive.gyro_reader_->GetAngle(),
        4.3 * rad / s, 270 * deg / s / s);
    std::cout << target_angle - subsystems_.drive.gyro_reader_->GetAngle() << " lolnope" << std::endl;
    subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
                                          std::move(angle_profile));
  }
  gyro_history_.Update(subsystems_.drive.gyro_reader_->GetAngle());
  return false;
}

void CitrusVision::EndTest() {
  // test_log_["end"] = std::to_string(table_->GetNumber("angleToTarget", 0));
  // test_log_.EndTest();
}
