#include "vision.h"
std::shared_ptr<NetworkTable> table=nullptr;
DrivetrainGoal runAlignment() {
        if(table==nullptr) {
                table=NetworkTable::GetTable("vision");
        }
        bool found=table->GetBoolean("targetFound");
        double error=table->GetNumber("angleToTarget");
        DrivetrainGoal goal;
        goal.steering=-error/30;
        goal.throttle=1;
        goal.highgear=false;
        goal.quickturn=true;
        goal.control_loop_driving=false;
        return goal;
}

