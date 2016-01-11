#include "drivetrain_subsystem.h"

using mutex_lock = std::lock_guard<std::mutex>;

DrivetrainSubsystem::DrivetrainSubsystem() : Updateable(200 * hz) {
  drive_ = std::make_unique<RobotDrive>(RobotPorts::drive_left,
                                        RobotPorts::drive_right);

  drive_loop_ = std::make_unique<DrivetrainLoop>();

  left_encoder_ = std::make_unique<Encoder>(RobotPorts::left_encoder_a,
                                            RobotPorts::left_encoder_b);
  right_encoder_ = std::make_unique<Encoder>(RobotPorts::right_encoder_a,
                                             RobotPorts::right_encoder_b);

  shifting_ = std::make_unique<DoubleSolenoid>(RobotPorts::shift_a,
                                               RobotPorts::shift_b);

  gyro_reader_ = std::make_unique<GyroReader>();
}

DrivetrainSubsystem::~DrivetrainSubsystem() {}

void DrivetrainSubsystem::Start() {
  Updateable::Start();
  drive_->SetSafetyEnabled(false);
  gyro_reader_->Start();
  is_operator_controlled_ = true;
  t = 0 * s;
}

void DrivetrainSubsystem::Update(Time dt) {
  DrivetrainPosition pos;
  DrivetrainOutput out;
  DrivetrainStatus status;

  SetDrivePosition(&pos);

  {
    mutex_lock lock(mu_);
    if (is_operator_controlled_) {
      drive_loop_->RunIteration(&current_goal_, &pos, &out, &status);
    } else {
      t += dt;
      Angle target_angle_ = angle_profile_->calculate_distance(t);
      Length target_distance_ = distance_profile_->calculate_distance(t);
      // TODO (Kyle): Track the targets here
    }
  }

  // TODO (Finn): Also deal with shifting output and with logging
  // from the status.
  drive_->TankDrive(out.left_voltage / 12.0, out.right_voltage / 12.0, false);
  shifting_->Set(current_goal_.highgear ? DoubleSolenoid::Value::kForward
                                        : DoubleSolenoid::Value::kReverse);
}

void DrivetrainSubsystem::SetDriveGoal(const DrivetrainGoal& goal) {
  mutex_lock lock(mu_);
  current_goal_ = goal;
}

void DrivetrainSubsystem::SetDrivePosition(
    DrivetrainPosition* drivetrain_position) {
  double click =
      3.14159 * .1016 / 360.0;  // Translating encoders into ground distances.
  drivetrain_position->left_encoder =
      left_encoder_->Get() * click;  // TODO (Ash): Get this from
                                     // the encoders in the right
                                     // units and direction.
  drivetrain_position->right_encoder = -right_encoder_->Get() * click;
  drivetrain_position->gyro_angle = gyro_reader_->GetAngle().to(rad);
  drivetrain_position->left_shifter_high = current_goal_.highgear;
  drivetrain_position->right_shifter_high = current_goal_.highgear;
}

void DrivetrainSubsystem::FollowMotionProfile(
    std::unique_ptr<MotionProfile<Length>> profile) {
  distance_profile_ = std::move(profile);
}

bool DrivetrainSubsystem::IsProfileComplete() {
  return distance_profile_ || angle_profile_;
}

void DrivetrainSubsystem::CancelMotionProfile() {
  mutex_lock lock(mu_);
  distance_profile_.release();
  angle_profile_.release();
}
