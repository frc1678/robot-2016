#include "arm_subsystem.h"
#include "frc1678/robot_ports.h"
#include "muan/utils/math_utils.h"
#include "robot_constants/robot_constants.h"

ArmSubsystem::ArmSubsystem()
    : muan::Updateable(200 * hz),
      pivot_controller_(RobotConstants::GetInstance()),
      elevator_controller_(RobotConstants::GetInstance(), .005 * s),
      csv_log_("arm_subsystem",
               {"time", "pivot_voltage", "elevator_voltage", "pivot_angle",
                "elevator_position", "state", "climb_state", "shooter_voltage",
                "shooter_velocity"}) {
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

  ball_sensor_ = std::make_unique<DigitalInput>(RobotPorts::ball_sensor);

  shot_timer_.Start();

  thresh_ = 0.5 * deg;

  intake_target_ = IntakeGoal::OFF;
}

ArmSubsystem::~ArmSubsystem() {}

bool ArmSubsystem::IsCalibrated() { return pivot_controller_.IsCalibrated(); }

void ArmSubsystem::Update(Time dt) {
  Voltage elevator_voltage = elevator_controller_.Update(
      dt, elevator_encoder_->Get() * .0003191764 * m,
      pivot_encoder_->Get() * deg / 5.0, enabled_);
  Voltage pivot_voltage = pivot_controller_.Update(
      dt, pivot_encoder_->Get() * deg / 5.0, !pivot_hall_->Get(), enabled_);
  bool elevator_brake = elevator_controller_.ShouldFireBrake(),
       pivot_brake = pivot_controller_.ShouldFireBrake();
  if (!enabled_) state_ = ArmState::DISABLED;
  switch (state_) {
    case ArmState::DISABLED:
      if (enabled_) state_ = ArmState::FINISHED;
      pivot_voltage = elevator_voltage = 0 * V;
      pivot_brake = pivot_controller_.ShouldFireBrake();
      elevator_brake = elevator_controller_.IsDone();
      break;
    case ArmState::RETRACTING:
      finished_ = false;
      pivot_voltage = 0 * V;
      if (elevator_controller_.IsDone()) {
        state_ = ArmState::MOVING_PIVOT;
        pivot_controller_.SetGoal(current_goal_.pivot_goal, thresh_);
      }
      break;
    case ArmState::MOVING_PIVOT:
      finished_ = false;
      elevator_voltage = 0 * V;
      elevator_brake = true;
      if (pivot_controller_.IsDone()) {
        state_ = ArmState::EXTENDING;
        elevator_controller_.SetGoal(current_goal_.elevator_goal);
      }
      shooter_controller_.SetGoal(current_goal_.shooter_goal);
      break;
    case ArmState::EXTENDING:
      finished_ = false;
      pivot_voltage = 0 * V;
      if (elevator_controller_.IsDone()) {
        state_ = ArmState::FINISHED;
      }
      break;
    case ArmState::FINISHED:
      if (pivot_controller_.IsCalibrated()) {
        pivot_voltage = 0 * V;
        pivot_brake = true;
        elevator_brake = true;
      }
      finished_ = true;
      elevator_voltage = 0 * V;
      break;
    case ArmState::CLIMBING:
      std::tie(pivot_voltage, pivot_brake, elevator_voltage, elevator_brake) =
          UpdateClimb(dt);
      break;
    case ArmState::ESTOP:
      pivot_voltage = elevator_voltage = 0 * V;
      pivot_brake = elevator_brake = false;
      break;
  }

  pivot_motor_a_->Set(pivot_voltage.to(12 * V));
  pivot_motor_b_->Set(pivot_voltage.to(12 * V));
  pivot_disk_brake_->Set(pivot_brake ? DoubleSolenoid::Value::kReverse
                                     : DoubleSolenoid::Value::kForward);
  elevator_motor_a_->Set(-elevator_voltage.to(12 * V));
  elevator_motor_b_->Set(-elevator_voltage.to(12 * V));
  elevator_disk_brake_->Set(elevator_brake ? DoubleSolenoid::Value::kReverse
                                           : DoubleSolenoid::Value::kForward);

  Voltage shooter_voltage =
      shooter_controller_.Update(0.005 * s, shooter_encoder_->Get() * deg);
  shooter_motor_a_->Set(
      state_ != ArmState::DISABLED ? -shooter_voltage.to(12 * V) : 0.0);
  shooter_motor_b_->Set(
      state_ != ArmState::DISABLED ? shooter_voltage.to(12 * V) : 0.0);

  // Yes, this code is structured weirdly. Yes, it is necessary because of
  // possible multithreading races
  if (should_shoot_) {
    intake_front_->Set(-1);
    intake_side_->Set(1);
    if (shot_timer_.Get() > shot_time) {
      should_shoot_ = false;
      shooter_controller_.SetGoal(0 * rad / s);
      SetHoodOpen(false);
    }
  } else if (intake_target_ == IntakeGoal::FORWARD_UNTIL) {
    intake_front_->Set(-1);
    intake_side_->Set(1);
    if (ball_sensor_->Get()) {
      intake_target_ = IntakeGoal::OFF;
    }
  } else if (intake_target_ == IntakeGoal::FORWARD_FOREVER) {
    intake_front_->Set(-1);
    intake_side_->Set(1);
  } else if (intake_target_ == IntakeGoal::REVERSE) {
    intake_front_->Set(1);
    intake_side_->Set(0);
  } else {
    intake_front_->Set(0);
    intake_side_->Set(0);
  }

  SmartDashboard::PutNumber("pivot_angle",
                            pivot_controller_.GetAngle().to(deg));

  csv_log_["time"] = std::to_string(t.to(s));
  csv_log_["pivot_voltage"] = std::to_string(pivot_voltage.to(V));
  csv_log_["elevator_voltage"] = std::to_string(elevator_voltage.to(V));
  csv_log_["pivot_angle"] =
      std::to_string(pivot_controller_.GetAngle().to(deg));
  csv_log_["elevator_position"] =
      std::to_string(elevator_controller_.GetPosition().to(m));
  csv_log_["state"] = std::to_string(static_cast<int>(state_));
  csv_log_["climb_state"] = std::to_string(static_cast<int>(climb_state_));
  csv_log_["shooter_voltage"] = std::to_string(shooter_voltage.to(V));
  csv_log_["shooter_velocity"] =
      std::to_string((shooter_controller_.GetVelocity()).to(rev / (60 * s)));
  csv_log_.EndLine();
  t += dt;
}

bool ArmSubsystem::BallIntaked() { return ball_sensor_->Get(); }

std::tuple<Voltage, bool, Voltage, bool> ArmSubsystem::UpdateClimb(Time dt) {
  Voltage elevator_voltage, pivot_voltage;
  bool elevator_brake, pivot_brake;
  elevator_voltage = elevator_controller_.UpdateClimb(
      dt, elevator_encoder_->Get() * .0003191764 * m,
      pivot_encoder_->Get() * deg / 5.0, enabled_);
  pivot_voltage = pivot_controller_.UpdateClimb(
      dt, pivot_encoder_->Get() * deg / 5.0, !pivot_hall_->Get(), enabled_);
  switch (climb_state_) {
    case ClimbState::PULLING_UP:
      pivot_brake = true;
      elevator_brake = elevator_controller_.ShouldFireBrake();
      if (elevator_controller_.IsDone()) {
        pivot_controller_.SetGoal(90 * deg, thresh_);
        climb_state_ = ClimbState::PIVOTING_ROBOT;
      }
      climbing_done_ = false;
      break;
    case ClimbState::PIVOTING_ROBOT:
      pivot_brake = pivot_controller_.ShouldFireBrake();
      elevator_brake = true;
      pivot_voltage = -12 * V;
      // pivot_voltage = pivot_controller_.UpdateClimb(
      //    dt, pivot_encoder_->Get() * deg / 5.0, !pivot_hall_->Get(),
      //    enabled_);
      if (pivot_controller_.IsDone()) {
        elevator_controller_.SetGoal(0 * m);
        climb_state_ = ClimbState::DONE;
      }
      climbing_done_ = false;
      break;
    case ClimbState::DONE:
      pivot_brake = elevator_brake = climbing_done_ = true;
      pivot_voltage = elevator_voltage = 0 * V;
      break;
  }
  return std::make_tuple(pivot_voltage, pivot_brake, elevator_voltage,
                         elevator_brake);
}

bool ArmSubsystem::IsDone() { return state_ == ArmState::FINISHED; }

// Sets targets for the arm subsystem
void ArmSubsystem::GoToLong() {
  ArmGoal goal{46 * deg, .38 * m, 5500 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::GoToAutoShot() {
  ArmGoal goal{36.5 * deg, 0 * m, 5500 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::GoToTuck() {
  ArmGoal goal{0 * deg, 0 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToTuckSpin() {
  ArmGoal goal{0 * deg, 0 * m, 5500 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToIntakeSpin() {
  ArmGoal goal{4.5 * deg, 0 * m, 5500 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToFender() {
  ArmGoal goal{10 * deg, 0 * m, 5500 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::GoToIntake() {
  ArmGoal goal{4 * deg, 0 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToDefensive() {
  ArmGoal goal{30 * deg, 0 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::StartClimb() {
  ArmGoal goal{85 * deg, 0.58 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::ContinueClimb() {
  ArmGoal goal{97 * deg, 0.58 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
  // I'm sorry, future self. I know you're disappointed in me, but I'm too
  // lazy
  // to do this correctly. :'(
  // Don't retract the arm
  state_ = ArmState::MOVING_PIVOT;
  pivot_controller_.SetGoal(current_goal_.pivot_goal, 1.0 * deg);
  SetHoodOpen(true);
}

void ArmSubsystem::CompleteClimb() {
  state_ = ArmState::CLIMBING;
  climb_state_ = ClimbState::PULLING_UP;
  elevator_controller_.SetGoal(0 * m);
}

void ArmSubsystem::SetHoodOpen(bool open) { shooter_hood_->Set(open); }

void ArmSubsystem::SetEnabled(bool enabled) { enabled_ = enabled; }

void ArmSubsystem::SetIntake(IntakeGoal goal) { intake_target_ = goal; }

void ArmSubsystem::SetGoal(ArmGoal goal) {
  if (pivot_controller_.IsCalibrated() &&
      !(state_ == ArmState::DISABLED || state_ == ArmState::ESTOP)) {
    elevator_controller_.SetGoal(0 * m);
    state_ = ArmState::RETRACTING;
    current_goal_ = goal;
  }
  shooter_controller_.SetGoal(0 * rad / s);
}

void ArmSubsystem::Shoot() {
  shot_timer_.Reset();
  should_shoot_ = true;
}

bool ArmSubsystem::ShooterSpeeded() {
  return shooter_controller_.IsAtVelocity();
}

bool ArmSubsystem::AllIsDone() { return finished_; }

bool ArmSubsystem::ClimbIsDone() { return climbing_done_; }
