#include "vision.h"
#include <iostream>
double integral = 0;
std::shared_ptr<NetworkTable> table=nullptr;
DrivetrainGoal runAlignment() {
        if(table==nullptr) {
                table=NetworkTable::GetTable("vision");
        }
        bool found=table->GetBoolean("targetFound", false);
        double error=table->GetNumber("angleToTarget", 0);
        DrivetrainGoal goal;
        integral += error / 1000;
        goal.steering=found ? error/80 + integral : 0;
        goal.throttle=0;
        goal.highgear=false;
        goal.quickturn=true;
        goal.control_loop_driving=false;
        return goal;
}

