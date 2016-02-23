#ifndef ARM_PIVOT_PIVOT_CONTROLLER_H_
#define ARM_PIVOT_PIVOT_CONTROLLER_H_

#include "muan/control/average_filter_pid.h"

class PivotController {
 public:
  PivotController();
  ~PivotController();
  void SetGoal(Angle goal, Angle thresh_);
  Voltage Update(Time dt, Angle encoder_angle, bool min_hall_triggered,
                 bool enabled);
  Voltage UpdateClimb(Time dt, Angle encoder_angle, bool min_hall_triggered,
                      bool enabled);
  bool ShouldFireBrake();
  bool IsDone();
  bool IsCalibrated();
  Angle GetAngle() { return last_; }

 private:
  enum class PivotState {
    DISABLED = 0,
    CALIBRATING,
    PREP_MOVING,
    MOVING,
    PREP_STOP,
    FINISHED,
    ESTOP
  };
  muan::AverageFilterPidController<Angle, Voltage> controller_,
      climb_controller_;
  Angle goal_;
  Angle last_;
  Angle offset_;
  PivotState state_ = PivotState::DISABLED;

  Voltage GetFFVoltage(Angle a);
  Voltage GetClimbFFVoltage(Angle a);

  bool calibrated_ = false;
  const Time disk_brake_time = .15 * s;
  Time brake_timer_;

  Angle thresh_;
};

#endif /* ARM_PIVOT_PIVOT_CONTROLLER_H_ */
