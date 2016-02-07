#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>

CitrusVision::CitrusVision(RobotSubsystems& subs)
    : subsystems_(subs), gyro_history_(.02 * s) {
  table_ = NetworkTable::GetTable("vision");
}

void CitrusVision::Start() {
  Angle camera_diff = -table_->GetNumber("angleToTarget", 0) * deg;
  bool is_found = table_->GetBoolean("targetFound", false);
  std::cout << "FOUND: " << is_found << ", moving to " << camera_diff.to(rad)
            << std::endl;
  if (is_found) {
    using muan::TrapezoidalMotionProfile;
    auto distance_profile = std::make_unique<TrapezoidalMotionProfile<Length>>(
        0 * m, 10 * m / s, 10 * m / s / s);
    auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
        camera_diff, 240 * deg / s, 500 * deg / s / s);
    subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
                                          std::move(angle_profile));
  }
}

bool CitrusVision::Update(bool enabled) {
  /* Angle camera_diff = -table_->GetNumber("angleToTarget", 0) * deg; */
  /* /1* Time latency = table_->GetNumber("captureTime", -100) * s; *1/ */
  /* // latency = std::min(latency, 1.99 * s); */
  /* bool is_found = table_->GetBoolean("targetFound", false); */
  /* // Angle camera_diff = -SmartDashboard::GetNumber("angleToTarget", 0) *
   * deg; */
  /* // Time latency = SmartDashboard::GetNumber("captureTime", -100); */
  /* Angle target_angle = camera_diff +
   * subsystems_.drive.gyro_reader_->GetAngle(); */
  /* if (subsystems_.drive.IsProfileComplete() && */
  /*     /1* latency >= 0 * s && *1/ is_found && enabled) { */
  /*   if (std::abs((target_angle - subsystems_.drive.gyro_reader_->GetAngle())
   */
  /*                    .to(deg)) < 0.5) { */
  /*     printf("Finished vision: %f s\n", test_timer.Get().to(s)); */
  /*     return true; */
  /*   } */
  /*   using muan::TrapezoidalMotionProfile; */
  /*   auto distance_profile =
   * std::make_unique<TrapezoidalMotionProfile<Length>>( */
  /*       0 * m, 10 * m / s, 10 * m / s / s); */
  /*   auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
   */
  /*       target_angle - subsystems_.drive.gyro_reader_->GetAngle(), */
  /*       4.3 * rad / s, 270 * deg / s / s); */
  /*   subsystems_.drive.FollowMotionProfile(std::move(distance_profile), */
  /*                                         std::move(angle_profile)); */
  /* } */
  /* gyro_history_.Update(subsystems_.drive.gyro_reader_->GetAngle()); */
  /* return false; */
  std::cout << "Angle travelled: " << subsystems_.drive.gyro_reader_->GetAngle()
            << std::endl;
}

void CitrusVision::EndTest() {
  // test_log_["end"] = std::to_string(table_->GetNumber("angleToTarget", 0));
  // test_log_.EndTest();
}
