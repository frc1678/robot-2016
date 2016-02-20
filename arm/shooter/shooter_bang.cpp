#include "unitscpp/unitscpp.h"
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
  if (velocity < goal_) {
    out_voltage = 12 * V;
  }
  previous_displacement_ = displacement;
  if (goal_ == 0) {
    out_voltage = 0 * V;
  }
  last_ = velocity;
  return muan::Cap(out_voltage, -12 * V, 12 * V);
}

void ShooterBang::SetGoal(AngularVelocity goal) { goal_ = goal; }

bool ShooterBang::IsDone() {
  return muan::abs(goal_ - last_) < 20.0 * rad / s;
}
AngularVelocity ShooterBang::GetVelocity() { return last_; }
