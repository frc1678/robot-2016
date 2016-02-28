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
  bool IsAtVelocity();
  AngularVelocity GetVelocity();

 private:
  AngularVelocity goal_, last_;

  Angle previous_displacement_;
  Time time;

  bool up_to_speed;
  bool has_up_to_speed;
};

#endif
