#include "elevator_controller.h"
#include "elevator_constants.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/control_utils.h"
#include "muan/utils/math_utils.h"
#include <iostream>

ElevatorController::ElevatorController(const RobotConstants& constants, Time dt)
    : controller_(constants.elevator_gains),
      climb_controller_(60 * V / m, 10 * V / m / s, 0 * V / m * s) {
  std::cout << "Elevator gains: " << constants.elevator_gains.kP << ", "
            << constants.elevator_gains.kI << ", "
            << constants.elevator_gains.kD << std::endl;
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
    case ElevatorState::PREP_MOVING:
      out_voltage = 0 * V;
      brake_timer_ += dt;
      if (brake_timer_ > disk_brake_time) {
        state_ = ElevatorState::MOVING;
      }
      break;
    case ElevatorState::MOVING:
      out_voltage = controller_.Calculate(dt, (current_goal_ - displacement)) +
                    1.2 * std::sin(arm_angle.to(rad)) * V;
      if (muan::abs(current_goal_ - displacement) < .5 * cm) {
        brake_timer_ = 0 * s;
        state_ = ElevatorState::PREP_STOP;
      }
      break;
    case ElevatorState::PREP_STOP:
      out_voltage = 1.2 * std::sin(arm_angle.to(rad)) * V;
      brake_timer_ += dt;
      if (brake_timer_ > disk_brake_time) {
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
  return out_voltage;
}

Voltage ElevatorController::UpdateClimb(Time dt, Length displacement,
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
      out_voltage =
          climb_controller_.Calculate(dt, (current_goal_ - displacement)) -
          3 * V;
      out_voltage = muan::Cap(out_voltage, -12 * V, 12 * V);
      if (muan::abs(current_goal_ - displacement) < .2 * cm) {
        state_ = ElevatorState::FINISHED;
      }
      break;
    default:
      out_voltage = 0 * V;
      break;
  }
  current_displacement_ = displacement;
  return out_voltage + 1.2 * std::sin(arm_angle.to(rad)) * V;
}

void ElevatorController::SetGoal(Length goal) {
  if (muan::abs(goal - current_goal_) > .2 * cm &&
      state_ != ElevatorState::DISABLED && state_ != ElevatorState::ESTOP) {
    current_goal_ = goal;
    state_ = ElevatorState::PREP_MOVING;
    brake_timer_ = 0 * s;
    controller_.Reset();
  }
}

bool ElevatorController::IsDone() { return state_ == ElevatorState::FINISHED; }

bool ElevatorController::ShouldFireBrake() {
  return state_ == ElevatorState::FINISHED ||
         state_ == ElevatorState::PREP_STOP;
}
