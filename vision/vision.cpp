#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>
#include <memory>

/*
namespace CitrusVision {

double integral = 0;
double last_error = 0;
std::shared_ptr<NetworkTable> table=nullptr;
void start(DrivetrainSubsystem* ds) {
        if(table==nullptr) {
                table=NetworkTable::GetTable("vision");
        }
        using muan::TrapezoidalMotionProfile;
        ds->CancelMotionProfile();
        auto ap =
std::make_unique<TrapezoidalMotionProfile<Angle>>(-table->GetNumber("angleToTarget",
0)*deg, 0.27*1.5*rev/s, 270*deg/s/s);
        ds->FollowMotionProfile(std::make_unique<TrapezoidalMotionProfile<Length>>(0,
0, 0), std::move(ap));
        integral = 0;
        last_error = 0;
}

void runAlignment(DrivetrainSubsystem* ds) {
        bool found=table->GetBoolean("targetFound", false);
        double error=table->GetNumber("angleToTarget", 0);
        DrivetrainGoal goal;
        integral += error / 10000;
        double derivative = (error - last_error) / .02;
        last_error = error;
        std::cout << integral << std::endl;
        goal.steering=found ? error/200 + integral + derivative / 1000: 0;
        goal.throttle=0;
        goal.highgear=false;
        goal.quickturn=true;
        goal.control_loop_driving=false;
        ds->SetDriveGoal(goal);
}

void updateVision(DrivetrainSubsystem* ds) {
  if (ds->IsProfileComplete()) {
    runAlignment(ds);
  }
}

}*/

CitrusVision::CitrusVision(RobotSubsystems& subs)
    : subsystems_(subs),
      turn_controller_(12 * V / rad, 0 * V / (rad * s), 0 * V / (rad / s)) {
  table_ = NetworkTable::GetTable("vision");
}

void CitrusVision::Start() {
  using muan::TrapezoidalMotionProfile;
  auto distance_profile =
      std::make_unique<TrapezoidalMotionProfile<Length>>(0, 0, 0);
  auto angle_profile = std::make_unique<TrapezoidalMotionProfile<Angle>>(
      -table_->GetNumber("angleToTarget", 0) * deg, 0.27 * 1.5 * rev / s,
      270 * deg / s / s);
  subsystems_.drive.FollowMotionProfile(std::move(distance_profile), std::move(angle_profile));
}

bool CitrusVision::Update() {
  if (subsystems_.drive.IsProfileComplete()) {
    // Run PID alignment
    Angle error = table_->GetNumber("angleToTarget") * deg;
    Voltage voltage = turn_controller_.Calculate(.02*s, error);
    DrivetrainGoal goal;
    bool found = table_->GetBoolean("targetFound");
    goal.steering = found ? voltage.to(12*V) : 0;
    goal.throttle = 0;
    goal.highgear = false;
    goal.quickturn = true;
    goal.control_loop_driving = false;
    subsystems_.drive.SetDriveGoal(goal);
  }
}
