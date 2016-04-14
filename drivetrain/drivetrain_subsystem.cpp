#include "drivetrain_subsystem.h"

using mutex_lock = std::lock_guard<std::mutex>;

// this code is an ugly hack but thats ok because the world is an ugly hack.

DrivetrainSubsystem::DrivetrainSubsystem()
    : muan::Updateable(200 * hz),
      angle_controller_(RobotConstants::GetInstance().drivetrain_angle_gains),
      distance_controller_(
          RobotConstants::GetInstance().drivetrain_distance_gains),
      event_log_("drivetrain_subsystem"),
      csv_log_("drivetrain_subsystem", {"enc_left", "enc_right", "goal_dist", "pwm_left",
                                        "pwm_right", "gyro_angle", "gear"}),
      csv_helper_(&csv_log_) {
  event_log_.Write("Initializing drivetrain subsystem components...", "INIT",
                   CODE_STAMP);
  drive_ = std::make_unique<RobotDrive>(RobotPorts::drive_left,
                                        RobotPorts::drive_right);

  drive_loop_ = std::make_unique<DrivetrainLoop>();

  left_encoder_ = std::make_unique<Encoder>(RobotPorts::left_encoder_a,
                                            RobotPorts::left_encoder_b);
  right_encoder_ = std::make_unique<Encoder>(RobotPorts::right_encoder_a,
                                             RobotPorts::right_encoder_b);

  shifting_ = std::make_unique<Solenoid>(RobotPorts::shift);

  gyro_reader_ = std::make_unique<GyroReader>();

  event_log_.Write("Done initializing drivetrain subsystem components", "INIT",
                   CODE_STAMP);

  current_goal_ = {0.0, 0.0, false, false, false, 0.0, 0.0, 0.0, 0.0};
}

DrivetrainSubsystem::~DrivetrainSubsystem() {}

void DrivetrainSubsystem::Start() {
  event_log_.Write("Starting drivetrain subsystem...", "INIT", CODE_STAMP);
  drive_->SetSafetyEnabled(false);
  gyro_reader_->Start();
  is_operator_controlled_ = true;
  t = 0 * s;
  Updateable::Start();  // This needs to be called last so that Update(Time)
                        // doesn't get called until after everything is
                        // initialized
  event_log_.Write("Drivetrain subsystem started", "INIT", CODE_STAMP);
}

void DrivetrainSubsystem::Update(Time dt) {
  DrivetrainPosition pos{0.0, 0.0, 0.0, false, false};
  DrivetrainOutput out{0.0, 0.0, false, false};
  DrivetrainStatus status{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false};

  SetDrivePosition(&pos);

  {
    mutex_lock lock(mu_);
    if (is_operator_controlled_) {
      current_goal_.control_loop_driving = false;
      drive_loop_->RunIteration(&current_goal_, &pos, 
                                 is_enabled_ ? &out : nullptr, &status); 
      out.left_voltage = -out.left_voltage;
      out.right_voltage = -out.right_voltage;
    }
    else {
      // TODO(Wesley) Add operator control to exit auto mode
      t += dt;

      // Feed forward term
      Voltage feed_forward_angle = GetAngleFFVoltage(
          angle_profile_->CalculateDerivative(t),
          angle_profile_->CalculateSecondDerivative(t), current_goal_.highgear);

      Voltage feed_forward_distance =
          GetDistanceFFVoltage(distance_profile_->CalculateDerivative(t),
                               distance_profile_->CalculateSecondDerivative(t),
                               current_goal_.highgear);

      // PID Term
      Angle target_angle_ = angle_profile_->Calculate(t);
      Length target_distance_ = distance_profile_->Calculate(t);

      // The difference between the gyro at the start of the profile and now
      Angle angle_from_start = gyro_reader_->GetAngle() - gyro_offset_;
      Length distance_from_start =
          (pos.left_encoder + pos.right_encoder) / 2 * m - encoder_offset_;

      // Gyro interpolation - TODO(Kyle or Wesley) Make this less sketchy
      Angle tmp_angle = angle_from_start;
      Angle tmp_old_angle = old_angle_;

      if (angle_from_start == old_angle_) {
        angle_from_start += (angle_from_start - even_older_angle_) / 2;
      }

      even_older_angle_ = tmp_old_angle;
      old_angle_ = tmp_angle;
      // End of gyro interpolation

      Voltage correction_angle =
          angle_controller_.Calculate(dt, target_angle_ - angle_from_start);
      Voltage correction_distance = distance_controller_.Calculate(
          dt, target_distance_ - distance_from_start);

      Voltage out_left = -feed_forward_angle - correction_angle +
                         feed_forward_distance + correction_distance;
      Voltage out_right = -feed_forward_angle + correction_angle +
                          feed_forward_distance + correction_distance;

      out.left_voltage = out_left.to(V);
      out.right_voltage = out_right.to(V);

      last_angle_ = angle_from_start;

      // End conditions
      bool profiles_finished_time =
          angle_profile_->finished(t) && distance_profile_->finished(t);
      bool profile_finished_distance =
          muan::abs(distance_from_start - distance_profile_->Calculate(t)) <
          4 * cm;
      bool profile_finished_angle =
          muan::abs(angle_from_start - angle_profile_->Calculate(t)) < 1 * deg;

      if (profiles_finished_time &&
          (profile_finished_angle || !use_angle_termination_) &&
          (profile_finished_distance || !use_distance_termination_)) {
        angle_profile_.reset();
        distance_profile_.reset();
        is_operator_controlled_ = true;
        angle_controller_.Reset();
        std::ostringstream ss;
        ss << "Finished motion profiles: " << angle_from_start.to(deg)
           << " degrees and " << distance_from_start.to(m) << " meters in "
           << t.to(s) << " seconds";
        event_log_.Write(ss.str(), "MOTION", CODE_STAMP);
      } else {
        csv_log_["goal_dist"] = std::to_string((distance_profile_->Calculate(t) + encoder_offset_).to(m));
      }
    }
  }

  // TODO (Finn): Also deal with shifting output and with logging
  // from the status.
  drive_->TankDrive(-out.left_voltage / 12.0, -out.right_voltage / 12.0, false);

  csv_log_["enc_left"] = std::to_string(pos.left_encoder);
  csv_log_["enc_right"] = std::to_string(pos.right_encoder);
  csv_log_["pwm_left"] = std::to_string(out.left_voltage);
  csv_log_["pwm_right"] = std::to_string(out.right_voltage);
  csv_log_["gyro_angle"] = std::to_string(pos.gyro_angle);
  csv_log_["gear"] = current_goal_.highgear ? "high" : "low";
  csv_helper_.Update();
  csv_log_.EndLine();  // Flush the current row of the log
}

void DrivetrainSubsystem::UpdateConstants() {
  RobotConstants::ReloadConstants();
  angle_controller_.SetGains(
      RobotConstants::GetInstance().drivetrain_angle_gains);
  distance_controller_.SetGains(
      RobotConstants::GetInstance().drivetrain_distance_gains);
}

void DrivetrainSubsystem::SetDriveGoal(const DrivetrainGoal &goal) {
  mutex_lock lock(mu_);
  Shift(current_goal_.highgear);
  current_goal_ = goal;
}

Length DrivetrainSubsystem::GetDistanceDriven() {
  DrivetrainPosition pos{0.0, 0.0, 0.0, false, false};
  SetDrivePosition(&pos);
  return (pos.left_encoder + pos.right_encoder) / 2 * m;
}

void DrivetrainSubsystem::Shift(bool high) {
  if (current_goal_.highgear != high) {
    shifting_->Set(!high);
    current_goal_.highgear = high;
  }
}

void DrivetrainSubsystem::SetDrivePosition(
    DrivetrainPosition *drivetrain_position) {
  double click =
      3.14159 * .1524 / 360.0;  // Translating encoders into ground distances.
  drivetrain_position->left_encoder =
      left_encoder_->Get() * click;  // TODO (Ash): Get this from the encoders
                                     // in the right units and direction.
  drivetrain_position->right_encoder = -right_encoder_->Get() * click;
  drivetrain_position->gyro_angle = gyro_reader_->GetAngle().to(rad);
  drivetrain_position->left_shifter_high = current_goal_.highgear;
  drivetrain_position->right_shifter_high = current_goal_.highgear;
}

// TODO(Wesley) Add generate motion profile functions so I don't repeat as much
// code

void DrivetrainSubsystem::PointTurn(Angle angle, bool highgear) { 
  AngularVelocity speed = (highgear ? 380 : 240) * deg / s;
  AngularAcceleration accel = (highgear ? 250 : 500) * deg / s / s;
  using muan::TrapezoidalMotionProfile;
  auto dp = std::make_unique<TrapezoidalMotionProfile<Length>>(
      0 * m, 10 * ft / s, 10 * ft / s / s);
  auto ap =
      std::make_unique<TrapezoidalMotionProfile<Angle>>(angle, speed, accel);
  FollowMotionProfile(std::move(dp), std::move(ap), highgear, true, false);
}


void DrivetrainSubsystem::AbsolutePointTurn(Angle angle, bool highgear) {
  AngularVelocity speed = (highgear ? 380 : 240) * deg / s;
  AngularAcceleration accel = (highgear ? 250 : 500) * deg / s / s;
  using muan::TrapezoidalMotionProfile;
  auto dp = std::make_unique<TrapezoidalMotionProfile<Length>>(
      0 * m, 10 * ft / s, 10 * ft / s / s);
  auto ap = std::make_unique<TrapezoidalMotionProfile<Angle>>(
      angle - gyro_reader_->GetAngle() - gyro_zero_offset_, speed, accel);
  FollowMotionProfile(std::move(dp), std::move(ap), highgear, true, false);
}

void DrivetrainSubsystem::DriveDistance(Length distance, bool highgear) {
  Velocity speed = (highgear ? 3.0 : 1.44) * m / s;
  Acceleration accel = 10 * ft / s / s;
  using muan::TrapezoidalMotionProfile;
  auto dp = std::make_unique<TrapezoidalMotionProfile<Length>>(distance, speed,
                                                               accel);
  auto ap = std::make_unique<TrapezoidalMotionProfile<Angle>>(
      0 * rad, 1 * rad / s, 1 * rad / s / s);
  FollowMotionProfile(std::move(dp), std::move(ap), highgear, true, true);
}

void DrivetrainSubsystem::DriveDistanceAtAngle(Length distance, Angle angle,
                                               bool highgear) {
  Velocity speed = (highgear ? 3.0 : 1.44) * m / s;
  Acceleration accel = 10 * ft / s / s;
  AngularVelocity angular_speed = (highgear ? 380 : 240) * deg / s;
  AngularAcceleration angular_accel = (highgear ? 250 : 500) * deg / s / s;
  using muan::TrapezoidalMotionProfile;
  auto dp = std::make_unique<TrapezoidalMotionProfile<Length>>(distance, speed,
                                                               accel);
  auto ap = std::make_unique<TrapezoidalMotionProfile<Angle>>(
      angle - gyro_reader_->GetAngle(), angular_speed, angular_accel);
  FollowMotionProfile(std::move(dp), std::move(ap), highgear, true, true);
}

void DrivetrainSubsystem::FollowMotionProfile(
    std::unique_ptr<muan::MotionProfile<Length>> distance_profile,
    std::unique_ptr<muan::MotionProfile<Angle>> angle_profile, bool highgear,
    bool use_distance_termination, bool use_angle_termination) {
  use_angle_termination_ = use_angle_termination;
  use_distance_termination_ = use_distance_termination;
  Shift(highgear);
  distance_profile_ = std::move(distance_profile);
  angle_profile_ = std::move(angle_profile);
  is_operator_controlled_ = false;
  t = 0 * s;
  DrivetrainPosition pos;
  SetDrivePosition(&pos);  // Is this bad?
  encoder_offset_ = (pos.left_encoder + pos.right_encoder) / 2;
  gyro_offset_ = gyro_reader_->GetAngle();
  angle_controller_.Reset();
  distance_controller_.Reset();
  is_loop_highgear = highgear;
}

bool DrivetrainSubsystem::IsProfileComplete() {
  return !(distance_profile_ && angle_profile_);
}

void DrivetrainSubsystem::CancelMotionProfile() {
  mutex_lock lock(mu_);
  distance_profile_.reset();
  angle_profile_.reset();
  is_operator_controlled_ = true;
}

Angle DrivetrainSubsystem::GetGyroAngle() { return gyro_reader_->GetAngle(); }

Voltage DrivetrainSubsystem::GetAngleFFVoltage(AngularVelocity velocity,
                                               AngularAcceleration acceleration,
                                               bool highgear) {
  // omega_dot = c1 * (V-c2*omega)
  // V = c2*omega - omega_dot / c1
  Voltage total_output;
  /* decltype((deg / s / s) / V) */
  if (highgear) {
    AngularVelocity max_robot_angular_velocity = 380 * deg / s;
    const auto c2 = 24 * V / max_robot_angular_velocity;
    const auto c1 = 70 * .75 / V * (deg / s / s);
    total_output = c2 * velocity + acceleration / c1;
  } else {
    AngularVelocity max_robot_angular_velocity = 240 * deg / s;
    const auto c2 = 24 * V / max_robot_angular_velocity;
    const auto c1 = 125 * .75 / V * (deg / s / s);
    total_output = c2 * velocity + acceleration / c1;
  }
  if (std::signbit(total_output.to(V)) != std::signbit(velocity.to(rad / s))) {
    total_output = 0;
  }
  return total_output / 2;
}

Voltage DrivetrainSubsystem::GetDistanceFFVoltage(Velocity velocity,
                                                  Acceleration acceleration,
                                                  bool highgear) {
  auto c1 = decltype(acceleration / V){0};
  auto c2 = decltype(V / velocity){0};
  if (highgear) {
    Velocity max_robot_velocity = 3.0 * m / s;
    c1 = .60 * .75 / V * (m / s / s);
    c2 = 22 * V / max_robot_velocity;
  } else {
    Velocity max_robot_velocity = 1.44 * m / s;
    c1 = 1 * .75 / V * (m / s / s);
    c2 = 24 * V / max_robot_velocity;
  }
  Voltage total_output = c2 * velocity + acceleration / c1;
  return total_output / 2;
}

void DrivetrainSubsystem::SetEnabled(bool enabled) { is_enabled_ = enabled; }
