#ifndef ARM_PIVOT_PIVOT_CONTROLLER_H_
#define ARM_PIVOT_PIVOT_CONTROLLER_H_

#include "muan/control/pid_controller.h"

class PivotController {
 public:
  PivotController();
  ~PivotController();
  void SetGoal(Angle goal);
  Voltage Update(Time dt, Angle encoder_angle, bool min_hall_triggered,
                 bool enabled);
  bool IsDone();

 private:
  enum class PivotState { DISABLED = 0, CALIBRATING, MOVING, FINISHED, ESTOP };
  muan::PidController<Angle, Voltage> controller_;
  Angle goal_;
  Angle offset_;
  PivotState state_ = PivotState::DISABLED;
  bool calibrated_ = false;
};

#endif /* ARM_PIVOT_PIVOT_CONTROLLER_H_ */
