#include "arm_subsystem.h"
#include "frc1678/robot_ports.h"

ArmSubsystem::ArmSubsystem() : muan::Updateable(200 * hz) {
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

  shooter_hood_ = std::make_unique<Solenoid>(RobotPorts::shooter_hood);
  intake_front_ = std::make_unique<VictorSP>(RobotPorts::intake_front);
  intake_side_ = std::make_unique<VictorSP>(RobotPorts::intake_side);
}

ArmSubsystem::~ArmSubsystem() {}

void ArmSubsystem::Update(Time dt) {
  Voltage v_pivot = pivot_controller_.Update(
      dt, pivot_encoder_->Get() * deg / 5.0, !pivot_hall_->Get(), enabled_);
  // std::cout << v_pivot.to(V) << std::endl;
  if (pivot_controller_.IsDone()) v_pivot = 0 * V;
  pivot_motor_a_->Set(v_pivot.to(12 * V));
  pivot_motor_b_->Set(v_pivot.to(12 * V));
  pivot_disk_brake_->Set(pivot_controller_.IsDone()
                             ? DoubleSolenoid::Value::kReverse
                             : DoubleSolenoid::Value::kForward);

  // shooter_motor_a_->Set(shooter_voltage_.to(12 * V));
}

void ArmSubsystem::GoToLong() {
  ArmGoal goal{45 * deg, 0 * m, 0 * rad / s};
  SetGoal(goal);
}

void ArmSubsystem::GoToTuck() {
  ArmGoal goal{0 * deg, 0 * m, 0 * rad / s};
  SetGoal(goal);
}

void ArmSubsystem::GoToFender() {
  ArmGoal goal{10 * deg, 0 * m, 0 * rad / s};
  SetGoal(goal);
}

void ArmSubsystem::GoToIntake() {
  ArmGoal goal{5 * deg, 0 * m, 0 * rad / s};
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
  shooter_motor_a_->Set(on ? -1. : 0.);
  shooter_motor_b_->Set(on ? 1. : 0.);
}

void ArmSubsystem::SetGoal(ArmGoal goal) {
  pivot_controller_.SetGoal(goal.pivot_goal);
}
