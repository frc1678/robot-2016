#include "pivot_controller.h"
#include "muan/utils/math_utils.h"
#include <iostream>

PivotController::PivotController()
    : controller_(20 * V / rad, 10 * V / (rad * s), 0 * V / (rad / s)) {
  goal_ = offset_ = 0 * deg;
}

PivotController::~PivotController() {}

void PivotController::SetGoal(Angle goal) {
  if (calibrated_) state_ = PivotState::MOVING;
  goal_ = goal;
}

Voltage PivotController::Update(Time dt, Angle encoder_angle,
                                bool min_hall_triggered, bool enabled) {
  Voltage out_voltage_;
  Angle angle = encoder_angle - offset_;
  if (!enabled) {
    state_ = PivotState::DISABLED;
  }
  switch (state_) {
    case PivotState::CALIBRATING:
      out_voltage_ = -1 * V;
      if (min_hall_triggered) {
        offset_ = encoder_angle - 20.2 * deg;
        state_ = PivotState::FINISHED;
        calibrated_ = true;
      }
      break;
    case PivotState::MOVING:
      out_voltage_ = controller_.Calculate(dt, goal_ - angle);
      std::cout << muan::abs(goal_ - angle).to(deg) << std::endl;
      if (muan::abs(goal_ - angle) < 1 * deg) {
        state_ = PivotState::FINISHED;
      }
      break;
    case PivotState::DISABLED:
      if (enabled) {
        state_ = (calibrated_ ? PivotState::FINISHED : PivotState::CALIBRATING);
      }
      out_voltage_ = 0 * V;
      break;
    case PivotState::FINISHED:
      std::cout << "FINISHED CALIBRATING" << std::endl;
      out_voltage_ = 0 * V;
      break;
    case PivotState::ESTOP:
      out_voltage_ = 0 * V;
      break;
  }
  out_voltage_ = muan::Cap(out_voltage_, -12 * V, 12 * V);
  return out_voltage_;
}

bool PivotController::IsDone() { return state_ == PivotState::FINISHED; }
