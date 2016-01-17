#include "vision.h"
NetworkTable* table=nullptr;
DriveTrainGoal runAlignment() {
        if(table==nullptr) {
                table=NetworkTable::GetTable("vision");
        }
        bool found=table->GetBoolean("targetFound");
        double error=table->GetNumber("angleToTarget");
        DriveTrainGoal goal;
        goal.steering=-error/30;
        goal.throttle=1;
        goal.highgear=false;
        goal.quickturn=true;
        goal.control_loop_driving=false;
        return goal;
}

