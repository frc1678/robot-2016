#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>

CitrusVision::CitrusVision(RobotSubsystems& subs)
    : subsystems_(subs),
      turn_controller_(10 * V / rad, 0 * V / (rad * s), 0 * V / (rad / s)) {
  table_ = NetworkTable::GetTable("vision");

}

void CitrusVision::Start() {
    Angle angle = -table_->GetNumber("angleToTarget", 0) * deg;
    using muan::TrapezoidalMotionProfile;
    auto distance_profile =
        std::make_unique<TrapezoidalMotionProfile<Length>>(0*m, 10*m/s, 10*m/s/s);
    auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(angle, 3.5*rad/s, 270*deg/s/s);
    subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
                                          std::move(angle_profile));
}

bool CitrusVision::Update() {
  Angle angle = -table_->GetNumber("angleToTarget", 0) * deg;
  if (std::abs(angle.to(deg)) < .2) {
    subsystems_.drive.CancelMotionProfile();
    std::cout << "VISION FINISHED " << angle.to(deg) << std::endl;
    return true;
  }
  else if (subsystems_.drive.IsProfileComplete()) {
    //if (std::abs((start_vision_angle_ - (start_gyro_angle_ - subsystems_.drive.gyro_reader_->GetAngle())).to(deg)) < 4) {//untested
      start_vision_angle_ = angle;
      start_gyro_angle_ = subsystems_.drive.gyro_reader_->GetAngle();

      using muan::TrapezoidalMotionProfile;
      auto distance_profile =
        std::make_unique<TrapezoidalMotionProfile<Length>>(0*m, 10*m/s, 10*m/s/s);
      auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(angle, 3.5*rad/s, 270*deg/s/s);
      subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
          std::move(angle_profile));
    //}
  }
  return false;
}

void CitrusVision::EndTest(){
  //test_log_["end"] = std::to_string(table_->GetNumber("angleToTarget", 0));
  //test_log_.EndTest();
}
