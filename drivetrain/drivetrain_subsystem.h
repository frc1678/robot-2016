#ifndef DRIVETRAIN_DRIVETRAIN_SUBSYSTEM_H_
#define DRIVETRAIN_DRIVETRAIN_SUBSYSTEM_H_

#include <memory>
#include <WPILib.h>

#include "muan/unitscpp/unitscpp.h"
#include "muan/multithreading/updateable.h"
#include "muan/control/motion_profile.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/control/pid_controller.h"
#include "muan/logging/text_log.h"
#include "muan/logging/csv_log.h"
#include "muan/utils/math_utils.h"
#include "frc1678/robot_ports.h"
#include "drivetrain/drivetrain.h"
#include "gyro/gyro_reader.h"
#include "utils/smart_dashboard_helper.h"
#include "robot_constants/robot_constants.h"

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
  void UpdateConstants();
  void SetDriveGoal(const DrivetrainGoal& goal);
  Length GetDistanceDriven();
  void Shift(bool high);

  void FollowMotionProfile(
      std::unique_ptr<muan::MotionProfile<Length>> distance_profile,
      std::unique_ptr<muan::MotionProfile<Angle>> angle_profile,
      bool highgear = false, bool use_distance_termination = true,
      bool use_angle_termination = true);
  bool IsProfileComplete();
  void CancelMotionProfile();

  void PointTurn(Angle angle, bool highgear = false);
  void AbsolutePointTurn(Angle angle, bool highgear = false);
  void DriveDistance(Length distance, bool highgear = false);
  void DriveDistanceAtAngle(Length distance, Angle angle,
                            bool highgear = false);

  void SetEnabled(bool enabled);

  Angle GetGyroAngle();

  std::unique_ptr<GyroReader>
      gyro_reader_;  // Made public so vision can access angle
 private:
  void SetDrivePosition(DrivetrainPosition* drivetrain_position);
  Voltage GetAngleFFVoltage(AngularVelocity velocity,
                            AngularAcceleration acceleration, bool highgear);
  Voltage GetDistanceFFVoltage(Velocity velocity, Acceleration acceleration,
                               bool highgear);

  std::unique_ptr<RobotDrive> drive_;
  std::unique_ptr<DrivetrainLoop> drive_loop_;
  std::unique_ptr<Encoder> left_encoder_, right_encoder_;
  std::unique_ptr<Solenoid> shifting_;

  bool is_operator_controlled_ = true;
  bool is_loop_highgear = true;

  bool is_enabled_ = false;

  Angle gyro_zero_offset_ = 0 * deg;

  DrivetrainGoal current_goal_;
  std::unique_ptr<muan::MotionProfile<Length>> distance_profile_;
  std::unique_ptr<muan::MotionProfile<Angle>> angle_profile_;

  bool use_distance_termination_ = true;
  bool use_angle_termination_ = true;

  std::atomic<bool> was_highgear{false};

  muan::PidController<Angle, Voltage> angle_controller_;
  muan::PidController<Length, Voltage> distance_controller_;

  Length encoder_offset_ = 0 * m;
  Angle gyro_offset_ = 0 * rad;
  Angle old_angle_ = 0 * rad;
  Angle even_older_angle_ = 0 * rad;
  Velocity last_angle_velocity_ = 0 * m / s;
  Velocity last_distance_velocity_ = 0 * m / s;

  Angle last_angle_ = 0 * rad;
  Time t;
  std::mutex mu_;

  muan::TextLog event_log_;
  muan::CSVLog csv_log_;
  SmartDashboardHelper csv_helper_;

  muan::Timer timer;
};

#endif
