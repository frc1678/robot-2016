#ifndef ARM_SHOOTER_SHOOTER_BANG_H_
#define ARM_SHOOTER_SHOOTER_BANG_H_

#include "muan/unitscpp/unitscpp.h"
#include "muan/utils/math_utils.h"

class ShooterBang {
 public:
  ShooterBang();
  ~ShooterBang();
  Voltage Update(Time dt, Angle displacement);
  void SetGoal(AngularVelocity goal);
  bool IsDone();

 private:
  AngularVelocity goal_, last_;
  Angle previous_displacement_;
  Time time; 
};

#endif
