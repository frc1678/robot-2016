#include "drivetrain_subsystem.h"

using mutex_lock = std::lock_guard<std::mutex>;

DrivetrainSubsystem::DrivetrainSubsystem()
    : muan::Updateable(200 * hz),
      angle_controller_(60*V/rad, 0*V/rad/s, 6*V/rad*s),
      distance_controller_(60*V/m, 0*V/m/s, 2.4*V/m*s), // UNTESTED PID CONSTANTS THESE WILL FUCK UP
      event_log_("drivetrain_subsystem"),
      csv_log_("drivetrain_subsystem", {"enc_left", "enc_right", "pwm_left",
                                        "pwm_right", "gyro_angle", "gear"}) {
  event_log_.Write("Initializing drivetrain subsystem components...", "INIT",
                   CODE_STAMP);
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

  event_log_.Write("Done initializing drivetrain subsystem components", "INIT",
                   CODE_STAMP);
}

DrivetrainSubsystem::~DrivetrainSubsystem() {}

void DrivetrainSubsystem::Start() {
  event_log_.Write("Starting drivetrain subsystem...", "INIT", CODE_STAMP);
  drive_->SetSafetyEnabled(false);
  gyro_reader_->Start();
  is_operator_controlled_ = false;//true;
  t = 0 * s;
  Updateable::Start();  // This needs to be called last so that Update(Time)
                        // doesn't get called until after everything is
                        // initialized
  event_log_.Write("Drivetrain subsystem started", "INIT", CODE_STAMP);
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
      //TODO(Wesley) Reset the PID controller if we went from tele to auto
      t += dt;
      Angle target_angle_ = angle_profile_->Calculate(t);
      Length target_distance_ = distance_profile_->Calculate(t);
      
      Voltage out_distance = angle_controller_.Calculate(dt, gyro_reader_->GetAngle() - target_angle_);
      Voltage out_angle = distance_controller_.Calculate(dt, ((left_encoder_->Get() + right_encoder_->Get())/2)*click - target_distance_);
    
      Voltage out_left = out_distance + out_angle;
      Voltage out_right = out_distance - out_angle;

      out.left_voltage = out_left.to(V);
      out.right_voltage = out_right.to(V);
    }
  }

  // TODO (Finn): Also deal with shifting output and with logging
  // from the status.
  drive_->TankDrive(out.left_voltage / 12.0, out.right_voltage / 12.0, false);
  shifting_->Set(current_goal_.highgear ? DoubleSolenoid::Value::kForward
                                        : DoubleSolenoid::Value::kReverse);

  csv_log_["enc_left"] = std::to_string(pos.left_encoder);
  csv_log_["enc_right"] = std::to_string(pos.right_encoder);
  csv_log_["pwm_left"] = std::to_string(out.left_voltage);
  csv_log_["pwm_right"] = std::to_string(out.right_voltage);
  csv_log_["gyro_angle"] = std::to_string(pos.gyro_angle);
  csv_log_["gear"] = current_goal_.highgear ? "high" : "low";
  csv_log_.EndLine();  // Flush the current row of the log
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
      left_encoder_->Get() * click;  // TODO (Ash): Get this from the encoders
                                     // in the right units and direction.
  drivetrain_position->right_encoder = -right_encoder_->Get() * click;
  drivetrain_position->gyro_angle = gyro_reader_->GetAngle().to(rad);
  drivetrain_position->left_shifter_high = current_goal_.highgear;
  drivetrain_position->right_shifter_high = current_goal_.highgear;
}

void DrivetrainSubsystem::FollowMotionProfile(
    std::unique_ptr<muan::MotionProfile<Length>> distance_profile,
    std::unique_ptr<muan::MotionProfile<Angle>> angle_profile) {
  distance_profile_ = std::move(distance_profile);
  angle_profile_ = std::move(angle_profile);
}

bool DrivetrainSubsystem::IsProfileComplete() {
  return !(distance_profile_ && angle_profile_);
}

void DrivetrainSubsystem::CancelMotionProfile() {
  mutex_lock lock(mu_);
  distance_profile_.release();
  angle_profile_.release();
}
