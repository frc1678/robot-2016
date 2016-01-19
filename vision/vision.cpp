#include "vision.h"
#include <iostream>
double integral = 0;
double last_error = 0;
std::shared_ptr<NetworkTable> table=nullptr;
void reset() {integral = 0;last_error = 0;}
DrivetrainGoal runAlignment() {
        if(table==nullptr) {
                table=NetworkTable::GetTable("vision");
        }
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
        return goal;
}

