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
  if(newDriveState) {
    robot->subsystems_.drive.DriveDistance(dist * ft);
    newDriveState = false;
  }


  if(robot->subsystems_.drive.IsProfileComplete()) { 
    newDriveState = true; 
    return true;
  }else{
    return false; 
  }

}

bool newDriveAtAngleState = true;
bool AutoFunction::DriveStraightAtAngle(CitrusRobot* robot, float angle, float dist) {
  if(newDriveState) {
    robot->subsystems_.drive.DriveDistanceAtAngle(dist * ft, angle * deg);
    newDriveState = false;
  }


  if(robot->subsystems_.drive.IsProfileComplete()) { 
    newDriveAtAngleState = true; 
    return true;
  }else{
    return false; 
  }

}



bool newTurnState = true;
bool AutoFunction::PointTurn(CitrusRobot* robot, float angle) {
  if(newTurnState) {
    robot->subsystems_.drive.PointTurn(angle * deg);
    newTurnState = false;
  }

  if(robot->subsystems_.drive.IsProfileComplete()) { 
    newTurnState = true; 
    return true;
  }else{
    return false; 
  }

}

bool newAbsTurnState = true;
bool AutoFunction::AbsolutePointTurn(CitrusRobot* robot, float angle) {
  if(newAbsTurnState) {
    robot->subsystems_.drive.AbsolutePointTurn(angle * deg);
    newAbsTurnState = false;
  }

  if(robot->subsystems_.drive.IsProfileComplete()) { 
    newAbsTurnState = true; 
    return true;
  }else{
    return false; 
  }

}

bool AutoFunction::Shift(CitrusRobot* robot, bool highgear) {
  robot->subsystems_.drive.Shift(highgear);
  return true;
}

bool AutoFunction::Wait(CitrusRobot* robot, float time) {
  return false;
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
