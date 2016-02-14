#include "elevator_controller.h"
#include "elevator_constants.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/control_utils.h"
#include "muan/utils/math_utils.h"
#include <iostream>

ElevatorController::ElevatorController(Time dt)
    : controller_(60 * V / m, 10 * V / m / s, 0 * V / m * s) {
  current_displacement_ = 0 * m;
  current_goal_ = 0 * m;
}

Voltage ElevatorController::Update(Time dt, Length displacement,
                                   Angle arm_angle, bool enabled) {
  if (!enabled) {
    state_ = ElevatorState::DISABLED;
  }
  Voltage out_voltage;
  switch (state_) {
    case ElevatorState::DISABLED:
      out_voltage = 0 * V;
      if (enabled) {
        state_ = ElevatorState::FINISHED;
      }
      break;
    case ElevatorState::MOVING:
      out_voltage = controller_.Calculate(dt, (current_goal_ - displacement));
      out_voltage = muan::Cap(out_voltage, -12 * V, 12 * V);
      if (muan::abs(current_goal_ - displacement) < .5 * cm) {
        state_ = ElevatorState::FINISHED;
      }
      break;
    case ElevatorState::FINISHED:
      out_voltage = 0 * V;
      break;
    case ElevatorState::ESTOP:
      out_voltage = 0 * V;
      break;
  }
  current_displacement_ = displacement;
  return out_voltage + 1.2 * std::sin(arm_angle.to(rad)) * V;
}

void ElevatorController::SetGoal(Length goal) {
  current_goal_ = goal;
  state_ = ElevatorState::MOVING;
  controller_.Reset();
}

bool ElevatorController::IsDone() { return state_ == ElevatorState::FINISHED; }
