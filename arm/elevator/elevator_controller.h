#ifndef ELEVATOR_CONTROLLER_H_
#define ELEVATOR_CONTROLLER_H_

#include "unitscpp/unitscpp.h"
#include "muan/control/state_observer.h"
#include "muan/control/state_feedback_controller.h"
#include "muan/logging/csv_log.h"
#include "muan/control/pid_controller.h"

class ElevatorController {
 public:
  ElevatorController(Time dt);
  Eigen::Matrix<double, 2, 1> GetObservedState();
  Voltage Update(Time dt, Length displacement, Angle arm_angle, bool enabled);
  void SetGoal(Length goal);
  bool IsDone();

  Length current_goal_;
  Length current_displacement_;

 private:
  enum class ElevatorState { DISABLED = 0, MOVING, FINISHED, ESTOP };
  ElevatorState state_ = ElevatorState::DISABLED;
  muan::PidController<Length, Voltage> controller_;
};

#endif
