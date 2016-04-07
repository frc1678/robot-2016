#ifndef ELEVATOR_CONTROLLER_H_
#define ELEVATOR_CONTROLLER_H_

#include "muan/unitscpp/unitscpp.h"
#include "muan/control/state_observer.h"
#include "muan/control/state_feedback_controller.h"
#include "muan/logging/csv_log.h"
#include "muan/control/pid_controller.h"
#include "robot_constants/robot_constants.h"

class ElevatorController {
 public:
  ElevatorController(const RobotConstants& constants, Time dt);
  Eigen::Matrix<double, 2, 1> GetObservedState();
  Voltage Update(Time dt, Length displacement, Angle arm_angle, bool enabled);
  Voltage UpdateClimb(Time dt, Length displacement, Angle arm_angle,
                      bool enabled);
  void SetGoal(Length goal);
  bool IsDone();
  bool ShouldFireBrake();

  Length GetPosition() { return current_displacement_; }
  Length GetError() { return (current_goal_ - current_displacement_); }

 private:
  enum class ElevatorState {
    DISABLED = 0,
    PREP_MOVING,
    MOVING,
    PREP_STOP,
    FINISHED,
    ESTOP
  };
  ElevatorState state_ = ElevatorState::DISABLED;
  muan::PidController<Length, Voltage> controller_;
  muan::PidController<Length, Voltage> climb_controller_;

  const Time disk_brake_time = .25 * s;
  Time brake_timer_ = 0 * s;

  Length current_goal_;
  Length current_displacement_;
};

#endif
