#include "auto_functions.h"

AutoFunction::AutoFunction()  {
 // Timer wait_timer = new Timer(); 
}

AutoFunction::~AutoFunction() {
 // delete wait_timer;
}

bool AutoFunction::Drivestraight(Length dist, Velocity speed) {
     
   return true;
}

bool AutoFunction::Turn(Angle angle, Velocity speed) {
  // this will be implemented when Wesley has his Auto drive stuff done
  return true;
}

bool AutoFunction::Wait(Time time) {
  if (true) {//wait_timer->Get() >= time){
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::Shoot(Position infield) {

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
}}

bool AutoFunction::RunIntake() {
 // intake->IntakePickup();
  return true;
}

bool AutoFunction::DropPinch() {
  return true;
}

bool AutoFunction::Align(Angle offset) {
  //to be implemented when vision works
  return true;
}

bool AutoFunction::StopDriving() {
 //driveSystem->DriveStraight(0,0);
 return true;
}
