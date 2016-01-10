#include <memory>
#include <WPILib.h>

#include "muan/multithreading/updateable.h"
#include "muan/control/motion_profile.h"
#include "frc1678/robot_ports.h"
#include "drivetrain/drivetrain.h"
#include "gyro/gyro_reader.h"

using drivetrain::control_loops::DrivetrainGoal;
using drivetrain::control_loops::DrivetrainPosition;
using drivetrain::control_loops::DrivetrainOutput;
using drivetrain::control_loops::DrivetrainStatus;
using drivetrain::control_loops::DrivetrainLoop;

class DrivetrainSubsystem : public Updateable {
 public:
  DrivetrainSubsystem();
  virtual ~DrivetrainSubsystem();

  void Update(Time dt) override;
  void Start();
  void SetDriveGoal(const DrivetrainGoal& goal);

  void FollowMotionProfile(std::unique_ptr<MotionProfile<Length>> profile);
  bool IsProfileComplete();
  void CancelMotionProfile();

 private:
  void SetDrivePosition(DrivetrainPosition* drivetrain_position);
  DrivetrainGoal current_goal_;
  std::unique_ptr<RobotDrive> drive_;
  std::unique_ptr<DrivetrainLoop> drive_loop_;
  std::unique_ptr<Encoder> left_encoder_, right_encoder_;
  std::unique_ptr<Joystick> j_wheel_, j_stick_;
  std::unique_ptr<DoubleSolenoid> shifting_;
  bool in_highgear_;
  std::unique_ptr<GyroReader> gyro_reader_;
  bool is_operator_controlled_ = true;
  std::unique_ptr<MotionProfile<Length>> distance_profile_;
  std::unique_ptr<MotionProfile<Angle>> angle_profile_;
  Time t;
};
