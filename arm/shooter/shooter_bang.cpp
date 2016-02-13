#include "shooter_bang.h"
#include <iostream>

ShooterBang::ShooterBang() {
  previous_displacement_ = 0 * deg;
  goal_ = 0 * (deg/s);
}

ShooterBang::~ShooterBang() {}

Voltage ShooterBang::Update(Time dt, Angle displacement) {
  Voltage out_voltage = 0 * V;
  AngularVelocity velocity = -((displacement - previous_displacement_)/(2*pi*dt));
  std::cout << velocity.to(rev/(60*s)) << ", " << (displacement - previous_displacement_).to(rev)<< std::endl; 
  if(velocity < goal_) {
    out_voltage = 12 * V;
  }
  previous_displacement_ = displacement;
  std::cout << out_voltage << std::endl;
  return muan::Cap(out_voltage, -12*V, 12*V);
}

void ShooterBang::SetGoal(AngularVelocity goal) {
  goal_ = goal;
}

