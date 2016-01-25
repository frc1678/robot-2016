#include "auto_functions.h"

#include <stdio.h>

void AutoFunction::SetUpAutoFunction() {
  // Timer wait_timer = new Timer();
}

void AutoFunction::DeleteAutoFunction() {
  // delete wait_timer;
}

bool AutoFunction::DriveStraight(RobotSubsystems* robot, Length dist,
                                 Velocity speed) {
  auto dp = std::make_unique<muan::TrapezoidalMotionProfile<Length>>(
      dist, speed, 10 * ft / s / s);
  auto ap = std::make_unique<muan::TrapezoidalMotionProfile<Angle>>(
      0 * deg, 50 * deg / s, 80 * deg / s / s);

  robot->drive.FollowMotionProfile(std::move(dp), std::move(ap));

  return true;
}


// Test for LemonScript
bool newDriveState = true;
bool AutoFunction::DriveStraight2(RobotSubsystems* robot, float dist,
                                  float speed) {
  if(newDriveState) {
    auto distanceProfile = std::make_unique<muan::TrapezoidalMotionProfile<Length>>(
      dist * ft, speed * ft / s, 10 * ft / s / s);
    auto angleProfile = std::make_unique<muan::TrapezoidalMotionProfile<Angle>>( 
      0 * deg, 50 * deg / s, 80 * deg / s / s);

    robot->drive.FollowMotionProfile(std::move(distanceProfile), std::move(angleProfile));
    newDriveState = false;
  }


  if(robot->drive.IsProfileComplete()) { 
    newDriveState = true; 
    return true;
  }else{
    return false; 
  }

}

bool AutoFunction::Turn(RobotSubsystems* robot, Angle angle, Velocity speed) {
  // this will be implemented when Wesley has his Auto drive stuff done
  return true;
}

bool AutoFunction::Wait(RobotSubsystems* robot, Time time) {
  if (true) {  // wait_timer->Get() >= time){
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::Shoot(RobotSubsystems* robot, Position infield) {
  if (infield == LOW_BAR) {
    // call shoot from low bar
  } else if (infield == BATTER) {
    // call shoof trom batter
  } else if (infield == WORKS_3) {
    // call shoof trom Outer Works 3
  } else if (infield == WORKS_4) {
    // call shoof trom Outer Works 4
  }

  return true;  // shooter->finished();
}

bool AutoFunction::RunIntake(RobotSubsystems* robot) {
  // intake->IntakePickup();
  return true;
}

bool AutoFunction::DropPinch(RobotSubsystems* robot) { return true; }

bool AutoFunction::Align(RobotSubsystems* robot, Angle offset) {
  // to be implemented when vision works
  return true;
}

bool AutoFunction::StopDriving(RobotSubsystems* robot) {
  // driveSystem->DriveStraight(0,0);
  return true;
}
