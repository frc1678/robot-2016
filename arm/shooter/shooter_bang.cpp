#include "unitscpp/unitscpp.h"
#include "shooter_bang.h"
#include <iostream>

ShooterBang::ShooterBang() {
  previous_displacement_ = 0 * deg;
  goal_ = 0 * (deg / s);
  velocity_ = 0 * (deg / s);
}

ShooterBang::~ShooterBang() {}

Voltage ShooterBang::Update(Time dt, Angle displacement) {
  Voltage out_voltage = 0 * V;
  velocity_ = -(displacement - previous_displacement_) / dt;
  if (velocity_ < goal_) {
    out_voltage = 12 * V;
  }
  previous_displacement_ = displacement;
  if (goal_ == 0) {
    out_voltage = 0 * V;
  }
  return muan::Cap(out_voltage, -12 * V, 12 * V);
}

void ShooterBang::SetGoal(AngularVelocity goal) { goal_ = goal; }

AngularVelocity ShooterBang::GetVelocity() { return velocity_; }
