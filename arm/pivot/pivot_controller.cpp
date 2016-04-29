#include "pivot_controller.h"
#include "muan/utils/math_utils.h"
#include <iostream>
#include <cmath>

PivotController::PivotController(RobotConstants constants)
    : controller_(constants.pivot_gains),
      climb_controller_(constants.pivot_climb_gains) {
  goal_ = offset_ = 0 * deg;
  thresh_ = .5 * deg;
  SetConstants(constants);
}

PivotController::~PivotController() {}

void PivotController::SetGoal(Angle goal, Angle thresh) {
  if (calibrated_) state_ = PivotState::MOVING;
  goal_ = goal;
  controller_.Reset();
  thresh_ = thresh;
}

Voltage PivotController::Update(Time dt, Angle encoder_angle,
                                bool min_hall_triggered, bool enabled) {
  Voltage out_voltage;
  Angle angle = encoder_angle - offset_;
  if (!enabled) {
    state_ = PivotState::DISABLED;
  }
  switch (state_) {
    case PivotState::CALIBRATING:
      out_voltage = -1 * V;
      if (min_hall_triggered) {
        offset_ = encoder_angle - calibration_offset_;
        state_ = PivotState::FINISHED;
        calibrated_ = true;
      }
      break;
    case PivotState::PREP_MOVING:
      out_voltage = 0 * V;
      brake_timer_ += dt;
      if (brake_timer_ > disk_brake_time) {
        state_ = PivotState::MOVING;
      }
      break;
    case PivotState::MOVING:
      out_voltage =
          controller_.Calculate(dt, goal_ - angle) + GetFFVoltage(angle);
      if (muan::abs(goal_ - angle) < thresh_ &&
          muan::abs(controller_.GetDerivative()) < 5 * deg / s) {
        state_ = PivotState::PREP_STOP;
        brake_timer_ = 0 * s;
      }
      break;
    case PivotState::PREP_STOP:
      out_voltage = GetFFVoltage(angle);
      brake_timer_ += dt;
      if (brake_timer_ > disk_brake_time) {
        state_ = PivotState::FINISHED;
      }
      break;
    case PivotState::DISABLED:
      if (min_hall_triggered && !calibrated_) {
        offset_ = encoder_angle - calibration_offset_;
        calibrated_ = true;
      }
      if (enabled) {
        state_ = (calibrated_ ? PivotState::FINISHED : PivotState::CALIBRATING);
      }
      out_voltage = 0 * V;
      break;
    case PivotState::FINISHED:
      out_voltage = 0 * V;
      break;
    case PivotState::ESTOP:
      out_voltage = 0 * V;
      break;
  }
  last_ = angle;
  out_voltage = muan::Cap(out_voltage, -4 * V, 12 * V);
  return out_voltage;
}

Voltage PivotController::UpdateClimb(Time dt, Angle encoder_angle,
                                     bool min_hall_triggered, bool enabled) {
  Voltage out_voltage;
  Angle angle = encoder_angle - offset_;
  if (!enabled) {
    state_ = PivotState::DISABLED;
  }
  switch (state_) {
    case PivotState::PREP_MOVING:
      state_ = PivotState::MOVING;
      out_voltage = 0 * V;
      tuck_timer_ = 0 * s;
      break;
    case PivotState::MOVING:
      if (goal_ != 0 * deg) {
        out_voltage = climb_controller_.Calculate(dt, goal_ - angle) +
                      GetClimbFFVoltage(angle);
        if (muan::abs(goal_ - angle) < thresh_) {
          state_ = PivotState::PREP_STOP;
          brake_timer_ = 0 * s;
        }
      } else {
        tuck_timer_ += dt;
        if (tuck_timer_ < 0.08 * s) {
          out_voltage = -12 * V;
        } else {
          state_ = PivotState::PREP_STOP;
          brake_timer_ = 0 * s;
        }
      }
      break;
    case PivotState::PREP_STOP:
      out_voltage = (goal_ != 0 * deg ? GetClimbFFVoltage(angle) : -8 * V);
      brake_timer_ += dt;
      if (brake_timer_ > disk_brake_time) {
        state_ = PivotState::FINISHED;
      }
      break;
    default:
      out_voltage = 0 * V;
      break;
  }
  last_ = angle;
  out_voltage = muan::Cap(out_voltage, -12 * V, 12 * V);

  return out_voltage;
}

void PivotController::RecalibratePivot() {
  offset_ = last_ + offset_;
}


bool PivotController::IsDone() { return state_ == PivotState::FINISHED; }

Voltage PivotController::GetFFVoltage(Angle a) {
  decltype(Force(0) * m) stall_torque = .71 * Force(1) * m;
  Current stall_current = 134 * A;
  Unitless gear_ratio = 1.0 / 609.52;

  auto motor_resistance = 12.0 * V / stall_current / 2.0;
  auto K_t = stall_torque / stall_current;

  auto grav = 9.81 * m / s / s;
  auto C_g = .57 * m;
  auto mass = 12 * kg;
  decltype(Force(0) * m) gravity_torque =
      C_g * mass * grav * std::cos(a.to(rad));
  return gravity_torque * gear_ratio * motor_resistance /
         (pivot_efficiency * K_t);
}

Voltage PivotController::GetClimbFFVoltage(Angle a) {
  decltype(Force(0) * m) stall_torque = .71 * Force(1) * m;
  Current stall_current = 134 * A;
  Unitless gear_ratio = 1.0 / 609.52;
  Unitless Q = .85;

  auto motor_resistance = 12.0 * V / stall_current / 2.0;
  auto K_t = stall_torque / stall_current;

  auto grav = 9.81 * m / s / s;
  auto C_g = .57 * m;
  auto mass = 60 * kg;
  decltype(Force(0) * m) gravity_torque =
      C_g * mass * grav * std::cos(a.to(rad));
  return gravity_torque * gear_ratio * motor_resistance / (Q * K_t);
}

bool PivotController::ShouldFireBrake() {
  return (IsDone() || state_ == PivotState::PREP_STOP) && calibrated_;
}

bool PivotController::IsCalibrated() { return calibrated_; }

void PivotController::SetConstants(RobotConstants constants) {
  calibrated_ = false;
  calibration_offset_ = constants.pivot_calibration_offset;
  pivot_efficiency = constants.pivot_efficiency;
}
