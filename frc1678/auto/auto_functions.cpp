#include "auto_functions.h"

AutoFunction::SetUpAutoFunction()  {
 // Timer wait_timer = new Timer(); 
}

AutoFunction::DeleteAutoFunction() {
 // delete wait_timer;
}

bool AutoFunction::Drivestraight(CitrusRobot* robot, Length dist, Velocity speed) {
   auto dp = std::make_unique<muan::TrapezoidalMotionProfile<Length>>(dist, speed, 10*ft/s/s);
   auto ap = std::make_unique<muan::TrapezoidalMotionProfile<Angle>>(0*deg, 50*deg/s, 80*deg/s); 

   robot->drive_subsystem_->FollowMotionProfile(std::move(dp), std::move(ap));

   return true;
}

bool AutoFunction::Turn(CitrusRobot* robot, Angle angle, Velocity speed) {
  // this will be implemented when Wesley has his Auto drive stuff done
  return true;
}

bool AutoFunction::Wait(CitrusRobot* robot, Time time) {
  if (true) {//wait_timer->Get() >= time){
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::Shoot(CitrusRobot* robot, Position infield) {

  if(infield  == LOW_BAR){
    //call shoot from low bar
  } else if(infield == BATTER) {
    //call shoof trom batter
  } else if(infield == WORKS_3) {
    //call shoof trom Outer Works 3
  } else if(infield == WORKS_4) {
    //call shoof trom Outer Works 4
  }
  
  return true;// shooter->finished();
}

bool AutoFunction::RunIntake(CitrusRobot* robot) {
 // intake->IntakePickup();
  return true;
}

bool AutoFunction::DropPinch(CitrusRobot* robot) {
  return true;
}

bool AutoFunction::Align(CitrusRobot* robot, Angle offset) {
  //to be implemented when vision works
  return true;
}

bool AutoFunction::StopDriving(CitrusRobot* robot) {
 //driveSystem->DriveStraight(0,0);
 return true;
}
