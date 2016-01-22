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
}

bool CitrusVision::Update() {
  if (subsystems_.drive.IsProfileComplete()) {
    std::cout << "REACHED" << std::endl;
    using muan::TrapezoidalMotionProfile;
    auto distance_profile =
        std::make_unique<TrapezoidalMotionProfile<Length>>(0, 0, 0);
    Angle angle = -table_->GetNumber("angleToTarget", 0) * deg;
    auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
        -table_->GetNumber("angleToTarget", 0) * deg, 0.27 * 1.5 * rev / s,
        270 * deg / s / s);
    subsystems_.drive.FollowMotionProfile(std::move(distance_profile),
                                          std::move(angle_profile));
    // std::cout << "REACHED" << std::endl;
    // Run PID alignment
    // Angle error = table_->GetNumber("angleToTarget") * deg;
    // Voltage voltage = turn_controller_.Calculate(.02*s, error);
    // DrivetrainGoal goal;
    // bool found = table_->GetBoolean("targetFound");
    // goal.steering = found ? voltage.to(12*V) : 0;
    // goal.throttle = 0;
    // goal.highgear = false;
    // goal.quickturn = true;
    // goal.control_loop_driving = false;
    // subsystems_.drive.SetDriveGoal(goal);
  }
}
