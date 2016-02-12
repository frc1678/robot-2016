#include "arm_subsystem.h"
#include "frc1678/robot_ports.h"
#include "muan/utils/math_utils.h"

ArmSubsystem::ArmSubsystem()
    : muan::Updateable(200 * hz), elevator_controller_(.005 * s) {
  pivot_encoder_ = std::make_unique<Encoder>(RobotPorts::pivot_encoder_a,
                                             RobotPorts::pivot_encoder_b);
  pivot_hall_ = std::make_unique<DigitalInput>(RobotPorts::pivot_hall);
  pivot_disk_brake_ = std::make_unique<DoubleSolenoid>(
      RobotPorts::pivot_brake_a, RobotPorts::pivot_brake_b);
  pivot_motor_a_ = std::make_unique<VictorSP>(RobotPorts::pivot_motor_a);
  pivot_motor_b_ = std::make_unique<VictorSP>(RobotPorts::pivot_motor_b);

  shooter_encoder_ = std::make_unique<Encoder>(RobotPorts::shooter_encoder_a,
                                               RobotPorts::shooter_encoder_b);
  shooter_motor_a_ = std::make_unique<VictorSP>(RobotPorts::shooter_a);
  shooter_motor_b_ = std::make_unique<VictorSP>(RobotPorts::shooter_b);

  elevator_encoder_ = std::make_unique<Encoder>(RobotPorts::elevator_encoder_a,
                                                RobotPorts::elevator_encoder_b);
  elevator_disk_brake_ = std::make_unique<DoubleSolenoid>(
      RobotPorts::elevator_brake_a, RobotPorts::elevator_brake_b);
  elevator_motor_a_ = std::make_unique<VictorSP>(RobotPorts::elevator_motor_a);
  elevator_motor_b_ = std::make_unique<VictorSP>(RobotPorts::elevator_motor_b);

  shooter_hood_ = std::make_unique<Solenoid>(RobotPorts::shooter_hood);
  intake_front_ = std::make_unique<VictorSP>(RobotPorts::intake_front);
  intake_side_ = std::make_unique<VictorSP>(RobotPorts::intake_side);
}

ArmSubsystem::~ArmSubsystem() {}

void ArmSubsystem::Update(Time dt) {
  Voltage elevator_voltage = elevator_controller_.Update(
      dt, elevator_encoder_->Get() * .0003191764 * m,
      pivot_encoder_->Get() * deg / 5.0, enabled_);
  Voltage pivot_voltage = pivot_controller_.Update(
      dt, pivot_encoder_->Get() * deg / 5.0, !pivot_hall_->Get(), enabled_);
  if (!enabled_) state_ = ArmState::DISABLED;
  switch (state_) {
    case ArmState::DISABLED:
      if (enabled_) state_ = ArmState::FINISHED;
      pivot_voltage = elevator_voltage = 0 * V;
      break;
    case ArmState::RETRACTING:
      pivot_voltage = 0 * V;
      if (elevator_controller_.IsDone()) {
        state_ = ArmState::MOVING_PIVOT;
        pivot_controller_.SetGoal(current_goal_.pivot_goal);
      }
      break;
    case ArmState::MOVING_PIVOT:
      elevator_voltage = 0 * V;
      if (pivot_controller_.IsDone()) {
        state_ = ArmState::EXTENDING;
        elevator_controller_.SetGoal(current_goal_.elevator_goal);
      }
      break;
    case ArmState::EXTENDING:
      pivot_voltage = 0 * V;
      if (elevator_controller_.IsDone()) {
        state_ = ArmState::FINISHED;
      }
      break;
    case ArmState::FINISHED:
      if (pivot_controller_.IsCalibrated()) {
        pivot_voltage = 0 * V;
      }
      elevator_voltage = 0 * V;
      break;
    case ArmState::ESTOP:
      pivot_voltage = elevator_voltage = 0 * V;
      break;
  }

  pivot_motor_a_->Set(pivot_voltage.to(12 * V));
  pivot_motor_b_->Set(pivot_voltage.to(12 * V));
  pivot_disk_brake_->Set(pivot_controller_.IsDone()
                             ? DoubleSolenoid::Value::kReverse
                             : DoubleSolenoid::Value::kForward);

  elevator_motor_a_->Set(-elevator_voltage.to(12 * V));
  elevator_motor_b_->Set(-elevator_voltage.to(12 * V));
  elevator_disk_brake_->Set(elevator_controller_.IsDone()
                                ? DoubleSolenoid::Value::kReverse
                                : DoubleSolenoid::Value::kForward);
}

void ArmSubsystem::GoToLong() {
  ArmGoal goal{44 * deg, .38 * m, 5500 * rev / (60 * s)};
  SetGoal(goal);
}

void ArmSubsystem::GoToTuck() {
  ArmGoal goal{0 * deg, 0 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
}

void ArmSubsystem::GoToFender() {
  ArmGoal goal{10 * deg, 0 * m, 4000 * rev / (60 * s)};
  SetGoal(goal);
}

void ArmSubsystem::GoToIntake() {
  ArmGoal goal{5 * deg, 0 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
}

void ArmSubsystem::SetHoodOpen(bool open) { shooter_hood_->Set(open); }

void ArmSubsystem::SetEnabled(bool enabled) { enabled_ = enabled; }

void ArmSubsystem::SetIntake(bool on) {
  intake_front_->Set(on ? -1. : 0.);
  intake_side_->Set(on ? 1. : 0.);
}

void ArmSubsystem::SetShooter(bool on) {
  SetHoodOpen(on);
  Voltage v_shooter =
      shooter_controller_.Update(0.005 * s, shooter_encoder_->Get() * deg);
  shooter_motor_a_->Set(on ? -v_shooter.to(12 * V) : 0.);
  shooter_motor_b_->Set(on ? v_shooter.to(12 * V) : 0.);
}

void ArmSubsystem::SetGoal(ArmGoal goal) {
  if (pivot_controller_.IsCalibrated() &&
      !(state_ == ArmState::DISABLED || state_ == ArmState::ESTOP)) {
    elevator_controller_.SetGoal(0 * m);
    state_ = ArmState::RETRACTING;
    current_goal_ = goal;
  }
  shooter_controller_.SetGoal(goal.shooter_goal);
}
