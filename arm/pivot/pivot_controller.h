#ifndef ARM_PIVOT_PIVOT_CONTROLLER_H_
#define ARM_PIVOT_PIVOT_CONTROLLER_H_

#include "muan/control/average_filter_pid.h"
#include "robot_constants/robot_constants.h"
#include "muan/unitscpp/unitscpp.h"

class PivotController {
 public:
  PivotController(RobotConstants constants);
  ~PivotController();
  void SetGoal(Angle goal, Angle thresh_);
  Voltage Update(Time dt, Angle encoder_angle, bool min_hall_triggered,
                 bool enabled);
  Voltage UpdateClimb(Time dt, Angle encoder_angle, bool min_hall_triggered,
                      bool enabled);
  void RecalibratePivot();
  bool ShouldFireBrake();
  bool IsDone();
  bool IsCalibrated();
  Angle GetAngle() { return last_; }
  Angle GetError() { return (goal_ - last_); }

 private:
  muan::AverageFilterPidController<Angle, Voltage> controller_,
      climb_controller_;
  void SetConstants(RobotConstants constants);
  enum class PivotState {
    DISABLED = 0,
    CALIBRATING,
    PREP_MOVING,
    MOVING,
    PREP_STOP,
    FINISHED,
    ESTOP,
  };
  Angle goal_;
  Angle last_;
  Angle offset_;
  PivotState state_ = PivotState::DISABLED;

  Voltage GetFFVoltage(Angle a);
  Voltage GetClimbFFVoltage(Angle a);

  bool calibrated_ = false;
  const Time disk_brake_time = .2 * s;
  Time brake_timer_;
  Time tuck_timer_;
  Angle calibration_offset_ = 0 * deg;

  double pivot_efficiency;

  Angle thresh_;
};

#endif /* ARM_PIVOT_PIVOT_CONTROLLER_H_ */
