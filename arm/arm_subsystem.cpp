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
  {
    Voltage v_pivot = pivot_controller_.Update(
        dt, pivot_encoder_->Get() * deg / 5.0, !pivot_hall_->Get(), enabled_);
    if (pivot_controller_.IsDone()) v_pivot = 0 * V;
    pivot_motor_a_->Set(v_pivot.to(12 * V));
    pivot_motor_b_->Set(v_pivot.to(12 * V));
    pivot_disk_brake_->Set(pivot_controller_.IsDone()
                               ? DoubleSolenoid::Value::kReverse
                               : DoubleSolenoid::Value::kForward);
  }

  {
    Voltage v_elevator = elevator_controller_.Update(
        dt, elevator_encoder_->Get() * .0003191764 * m, enabled_);
    v_elevator = muan::Cap(v_elevator, -12 * V, 12 * V);
    if (elevator_controller_.IsDone()) {
      v_elevator = 0 * V;
    }
    elevator_motor_a_->Set(-v_elevator.to(12 * V));
    elevator_motor_b_->Set(-v_elevator.to(12 * V));
    elevator_disk_brake_->Set(elevator_controller_.IsDone()
                                  ? DoubleSolenoid::Value::kReverse
                                  : DoubleSolenoid::Value::kForward);
  }
}

void ArmSubsystem::GoToLong() {
  ArmGoal goal{44 * deg, .38 * m, 5500 * rev / (60*s)};
  SetGoal(goal);
}

void ArmSubsystem::GoToTuck() {
  ArmGoal goal{0 * deg, 0 * m, 0 * rev / (60*s)};
  SetGoal(goal);
}

void ArmSubsystem::GoToFender() {
  ArmGoal goal{10 * deg, 0 * m, 4000 * rev / (60*s)};
  SetGoal(goal);
}

void ArmSubsystem::GoToIntake() {
  ArmGoal goal{5 * deg, 0 * m, 0 * rev / (60*s)};
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
  Voltage v_shooter = shooter_controller_.Update(0.005 * s , shooter_encoder_->Get() * deg);
  shooter_motor_a_->Set(on ? -v_shooter.to(12*V) : 0.);
  shooter_motor_b_->Set(on ? v_shooter.to(12*V) : 0.);
}

void ArmSubsystem::SetGoal(ArmGoal goal) {
  pivot_controller_.SetGoal(goal.pivot_goal);
  elevator_controller_.SetGoal(goal.elevator_goal);
  shooter_controller_.SetGoal(goal.shooter_goal);
}
