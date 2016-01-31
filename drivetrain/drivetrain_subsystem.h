#ifndef DRIVETRAIN_DRIVETRAIN_SUBSYSTEM_H_
#define DRIVETRAIN_DRIVETRAIN_SUBSYSTEM_H_

#include <memory>
#include <WPILib.h>

#include "muan/multithreading/updateable.h"
#include "muan/control/motion_profile.h"
#include "muan/control/pid_controller.h"
#include "muan/logging/text_log.h"
#include "muan/logging/csv_log.h"
#include "frc1678/robot_ports.h"
#include "drivetrain/drivetrain.h"
#include "gyro/gyro_reader.h"

using drivetrain::control_loops::DrivetrainGoal;
using drivetrain::control_loops::DrivetrainPosition;
using drivetrain::control_loops::DrivetrainOutput;
using drivetrain::control_loops::DrivetrainStatus;
using drivetrain::control_loops::DrivetrainLoop;

class DrivetrainSubsystem : public muan::Updateable {
 public:
  DrivetrainSubsystem();
  virtual ~DrivetrainSubsystem();

  void Update(Time dt) override;
  void Start();
  void SetDriveGoal(const DrivetrainGoal& goal);

  void FollowMotionProfile(
      std::unique_ptr<muan::MotionProfile<Length>> distance_profile,
      std::unique_ptr<muan::MotionProfile<Angle>> angle_profile);
  bool IsProfileComplete();
  void CancelMotionProfile();

  Angle GetGyroAngle();

  std::unique_ptr<GyroReader>
      gyro_reader_;  // Made public so vision can access angle
 private:
  void SetDrivePosition(DrivetrainPosition* drivetrain_position);

  std::unique_ptr<RobotDrive> drive_;
  std::unique_ptr<DrivetrainLoop> drive_loop_;
  std::unique_ptr<Encoder> left_encoder_, right_encoder_;
  std::unique_ptr<DoubleSolenoid> shifting_;

  bool in_highgear_;
  bool is_operator_controlled_ = true;

  DrivetrainGoal current_goal_;
  std::unique_ptr<muan::MotionProfile<Length>> distance_profile_;
  std::unique_ptr<muan::MotionProfile<Angle>> angle_profile_;

  muan::PidController<Angle, Voltage> angle_controller_;
  muan::PidController<Length, Voltage> distance_controller_;

  float encoder_offset_ = 0;
  Angle gyro_offset_ = 0 * rad;
  Angle old_angle_ = 0 * rad;
  Angle even_older_angle_ = 0 * rad;

  Angle last_angle_ = 0 * rad;
  Time t;
  std::mutex mu_;

  muan::TextLog event_log_;
  muan::CSVLog csv_log_;
};

#endif
