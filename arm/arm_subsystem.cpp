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
                "shooter_velocity"}),
      csv_helper_(&csv_log_) {
  pivot_encoder_ = std::make_unique<Encoder>(RobotPorts::pivot_encoder_a,
                                             RobotPorts::pivot_encoder_b);
  pivot_hall_ = std::make_unique<DigitalInput>(RobotPorts::pivot_hall);
  pivot_disk_brake_ = std::make_unique<DoubleSolenoid>(
      RobotPorts::pivot_brake_a, RobotPorts::pivot_brake_b);
  pivot_disk_brake_->Set(was_pivot_brake_ ? DoubleSolenoid::Value::kReverse
                                          : DoubleSolenoid::Value::kForward);
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
  elevator_disk_brake_->Set(was_elevator_brake_
                                ? DoubleSolenoid::Value::kReverse
                                : DoubleSolenoid::Value::kForward);
  elevator_motor_a_ = std::make_unique<VictorSP>(RobotPorts::elevator_motor_a);
  elevator_motor_b_ = std::make_unique<VictorSP>(RobotPorts::elevator_motor_b);

  shooter_hood_ = std::make_unique<Solenoid>(RobotPorts::shooter_hood);
  intake_front_ = std::make_unique<VictorSP>(RobotPorts::intake_front);
  intake_side_ = std::make_unique<VictorSP>(RobotPorts::intake_side);

  ball_pinch_ = std::make_unique<Solenoid>(RobotPorts::ball_pinch);

  ball_sensor_ = std::make_unique<DigitalInput>(RobotPorts::ball_sensor);

  camera_light_ = std::make_unique<DigitalOutput>(25);
  camera_light_->Set(1);

  shot_timer_.Start();

  thresh_ = 0.5 * deg;

  intake_target_ = IntakeGoal::OFF;
  intake_timer_ = 0 * s;

  constants = RobotConstants::GetInstance();
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
      if(climbing_advance_ && !pivot_controller_.IsDone()) {
        pivot_voltage = 6.0 * V;
      } else if (climbing_advance_ && pivot_controller_.IsDone()) {
        pivot_voltage = 0.0 * V;
      }
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
  SetPivotBrake(pivot_brake);

  elevator_motor_a_->Set(-elevator_voltage.to(12 * V));
  elevator_motor_b_->Set(-elevator_voltage.to(12 * V));
        
  if((ball_sensor_->Get() && climbing_advance_) && (pivot_controller_.GetError() < (3 * deg))){
    CompleteClimb();
    climbing_advance_ = false;
  }

  SetElevatorBrake(elevator_brake);

  Voltage shooter_voltage =
      shooter_controller_.Update(0.005 * s, shooter_encoder_->Get() * deg);
  shooter_voltage = (ball_sensor_->Get() || proxy_shot_override_ || proxy_position_override_) ? shooter_voltage : 0;
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
      if (intake_timer_ < 0.5 * s) {
        intake_front_->Set(-1);
        intake_side_->Set(1);
      } else {
        intake_target_ = IntakeGoal::OFF;
      }
      intake_timer_ += dt;
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
    intake_timer_ = 0 * s;
  }

  SmartDashboard::PutBoolean("advance climb", climbing_advance_);
                            

  csv_log_["time"] = std::to_string(t.to(s));
  csv_log_["climbing"] = std::to_string(climbing_advance_);
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
  csv_helper_.Update();
  csv_log_.EndLine();
  t += dt;
}

bool ArmSubsystem::BallIntaked() { return ball_sensor_->Get(); }

std::tuple<Voltage, bool, Voltage, bool> ArmSubsystem::UpdateClimb(Time dt) {
  Voltage elevator_voltage, pivot_voltage;
  climbing_advance_ = false;
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
        pivot_controller_.SetGoal(85 * deg, thresh_);
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
  ArmGoal goal =
      constants
          .long_shot_goals;  // Look at robot_constants to change these values
  proxy_position_override_ = true;
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::GoToAutoShot() {
  ArmGoal goal =
      constants
          .auto_shot_goals;  // Look at robot_constants to change these values
  proxy_position_override_ = true;
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::GoToTuck() {
  ArmGoal goal{0 * deg, 0 * m, 0 * rev / (60 * s)};
  proxy_position_override_ = false;
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToTuckSpin() {
  ArmGoal goal{0 * deg, 0 * m, 6500 * rev / (60 * s)};
  proxy_position_override_ = false;
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToIntakeSpin() {
  ArmGoal goal{3.0 * deg, 0 * m, 6500 * rev / (60 * s)};
  proxy_position_override_ = false;
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToFender() {
  ArmGoal goal =
      constants
          .fender_shot_goals;  // Look at robot_constants to change these values
  proxy_position_override_ = true;
  SetGoal(goal);
  SetHoodOpen(true);
}

void ArmSubsystem::GoToIntake() {
  ArmGoal goal{3.6 * deg, 0 * m, 0 * rev / (60 * s)};
  proxy_position_override_ = false;
  SetGoal(goal);
  SetHoodOpen(false);
}

void ArmSubsystem::GoToDefensive() {
  ArmGoal goal{30 * deg, 0 * m, 0 * rev / (60 * s)};
  proxy_position_override_ = false;
  SetGoal(goal);
  SetHoodOpen(false);
  climbing_advance_ = false;
}

void ArmSubsystem::GoToDefensiveSpin() {
  ArmGoal goal{30 * deg, 0 * m, 6500 * rev / (60 * s)};
  proxy_position_override_ = false;
  SetGoal(goal);
  SetHoodOpen(false);
  climbing_advance_ = false;
}


void ArmSubsystem::StartClimb() {
  ArmGoal goal{85 * deg, 0.58 * m, 0 * rev / (60 * s)};
  SetGoal(goal);
  SetHoodOpen(true);
  climbing_advance_ = false;
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
  climbing_advance_ = true;
}

void ArmSubsystem::CompleteClimb() {
  state_ = ArmState::CLIMBING;
  climb_state_ = ClimbState::PULLING_UP;
  elevator_controller_.SetGoal(0 * m);
  climbing_advance_ = false;
}

void ArmSubsystem::DropBall() {
  ball_pinch_->Set(true);
}

void ArmSubsystem::SetHoodOpen(bool open) {
  shooter_hood_->Set(open);
  camera_light_->Set(open);
}

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

void ArmSubsystem::Shoot(bool checkspeed) {
  shot_timer_.Reset();
  if(ShooterSpeeded() || !checkspeed) { should_shoot_ = true; }
}

bool ArmSubsystem::IsShooting() {
  return should_shoot_;
}

bool ArmSubsystem::ShooterSpeeded() {
  return shooter_controller_.IsAtVelocity();
}

bool ArmSubsystem::AllIsDone() { return finished_; }

bool ArmSubsystem::ClimbIsDone() { return climbing_done_; }

void ArmSubsystem::SetPivotBrake(bool on) {
  if (on != was_pivot_brake_) {
    pivot_disk_brake_->Set(on ? DoubleSolenoid::Value::kReverse
                              : DoubleSolenoid::Value::kForward);
    was_pivot_brake_ = on;
  }
}

void ArmSubsystem::SetElevatorBrake(bool on) {
  if (on != was_elevator_brake_) {
    elevator_disk_brake_->Set(on ? DoubleSolenoid::Value::kReverse
                                 : DoubleSolenoid::Value::kForward);
    was_elevator_brake_ = on;
  }
}
