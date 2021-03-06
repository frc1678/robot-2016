#include "muan/unitscpp/unitscpp.h"
#include "shooter_bang.h"
#include <iostream>

ShooterBang::ShooterBang() {
  previous_displacement_ = 0 * deg;
  goal_ = 0 * (deg / s);
  last_ = 0 * (deg / s);
}

ShooterBang::~ShooterBang() {}

Voltage ShooterBang::Update(Time dt, Angle displacement) {
  Voltage out_voltage = 0 * V;
  time += dt;
  AngularVelocity velocity = -(displacement - previous_displacement_) / dt;
  if (muan::abs(velocity) < goal_) {
    out_voltage = 12 * V;
  }
  previous_displacement_ = displacement;
  if (goal_ == 0) {
    out_voltage = 0 * V;
  }
  last_ = velocity;
  return muan::Cap(out_voltage, -12 * V, 12 * V);
}

void ShooterBang::SetGoal(AngularVelocity goal) {
  goal_ = goal;
  up_to_speed = false;
  has_up_to_speed = false;
}

bool ShooterBang::IsAtVelocity() {
  if (muan::abs(last_) > goal_ && !up_to_speed) {
    has_up_to_speed = true;
  }
  if (has_up_to_speed) {
    up_to_speed = false;
    return true;
  }
  return false;
}

bool ShooterBang::IsDone() { return muan::abs(goal_ - last_) < 20.0 * rad / s; }

AngularVelocity ShooterBang::GetVelocity() { return last_; }
