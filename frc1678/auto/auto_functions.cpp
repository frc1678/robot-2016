#include "auto_functions.h"

#include <stdio.h>

void AutoFunction::SetUpAutoFunction() {
  // Timer wait_timer = new Timer();
}

void AutoFunction::DeleteAutoFunction() {
  // delete wait_timer;
}


// Test for LemonScript
bool newDriveState = true;
bool AutoFunction::DriveStraight(CitrusRobot* robot, float dist) {
  printf("Fake driving straight\n");
  return true;

  if(newDriveState) {
    auto distanceProfile = std::make_unique<muan::TrapezoidalMotionProfile<Length>>(
      dist * ft, 12.0 * ft / s, 10 * ft / s / s);
    auto angleProfile = std::make_unique<muan::TrapezoidalMotionProfile<Angle>>( 
      0 * deg, 50 * deg / s, 80 * deg / s / s);

    robot->subsystems_.drive.FollowMotionProfile(std::move(distanceProfile), std::move(angleProfile));
    newDriveState = false;
  }


  if(robot->subsystems_.drive.IsProfileComplete()) { 
    newDriveState = true; 
    return true;
  }else{
    return false; 
  }

}



bool newTurnState = true;
bool AutoFunction::PointTurn(CitrusRobot* robot, float angle) {
  if(newTurnState) {

          
    printf("Starting point turn with angle = %f, speed = %f\n", angle, 240.0);
    auto distanceProfile = std::make_unique<muan::TrapezoidalMotionProfile<Length>>(
      0 * ft, 0 * ft / s, 0 * ft / s / s);
    auto angleProfile = std::make_unique<muan::TrapezoidalMotionProfile<Angle>>( 
      angle * deg, 240 * deg / s, 500 * deg / s / s);

    robot->subsystems_.drive.FollowMotionProfile(std::move(distanceProfile), std::move(angleProfile));
    newTurnState = false;
  }


  if(robot->subsystems_.drive.IsProfileComplete()) { 
    newTurnState = true; 
    return true;
  }else{
    return false; 
  }

}


bool AutoFunction::Wait(CitrusRobot* robot, Time time) {
  if (true) {  // wait_timer->Get() >= time){
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::Shoot(CitrusRobot* robot, Position infield) {
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

bool AutoFunction::RunIntake(CitrusRobot* robot) {
  // intake->IntakePickup();
  return true;
}

bool toSetPosition = true;
bool AutoFunction::SetArmPosition(CitrusRobot* robot, Position arm_position) {
  // (TODO) Ash: Set this up later.
  return true;
}

bool AutoFunction::DropPinch(CitrusRobot* robot) { return true; }

bool toStartVision = true;
bool AutoFunction::Align(CitrusRobot* robot) {
  if (toStartVision) {
    robot->vision_.Start();
    toStartVision = false;
  }

  if (robot->vision_.Update(true)) { // (TODO) Ash: Modify when vision Update() is fixed.
    toStartVision = true;
    return true;
  }

  return false;
}

bool AutoFunction::StopDriving(CitrusRobot* robot) {
  // driveSystem->DriveStraight(0,0);
  return true;
}
