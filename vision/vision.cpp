#include "vision.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include <iostream>

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
        auto ap = std::make_unique<TrapezoidalMotionProfile<Angle>>(-table->GetNumber("angleToTarget", 0)*deg, 0.27*1.5*rev/s, 270*deg/s/s);
        ds->FollowMotionProfile(std::make_unique<TrapezoidalMotionProfile<Length>>(0, 0, 0), std::move(ap));
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

}
