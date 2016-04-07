#ifndef ARM_ARM_SUBSYSTEM_H_
#define ARM_ARM_SUBSYSTEM_H_

#include <WPILib.h>
#include <memory>
#include "muan/multithreading/updateable.h"
#include "pivot/pivot_controller.h"
#include "elevator/elevator_controller.h"
#include "shooter/shooter_bang.h"
#include "muan/utils/timer.h"
#include "utils/smart_dashboard_helper.h"

enum class IntakeGoal { OFF = 0, REVERSE, FORWARD_UNTIL, FORWARD_FOREVER };

class ArmSubsystem : public muan::Updateable {
 public:
  ArmSubsystem();
  ~ArmSubsystem();

  bool IsCalibrated();

  void Update(Time dt) override;

  bool IsDone();

  void GoToLong();
  void GoToAutoShot();
  void GoToIntakeSpin();
  void GoToTuck();
  void GoToTuckSpin();
  void GoToFender();
  void GoToIntake();
  void GoToDefensive();

  void StartClimb();
  void ContinueClimb();
  void CompleteClimb();

  void SetEnabled(bool enabled);

  void SetIntake(IntakeGoal goal);
  void SetShooter(bool on);

  void Shoot();
  bool ShooterSpeeded();

  bool AllIsDone();
  bool ClimbIsDone();
  bool BallIntaked();

 private:
  std::tuple<Voltage, bool, Voltage, bool> UpdateClimb(Time dt);

  void SetHoodOpen(bool open);

  void SetGoal(ArmGoal goal);

  enum class ArmState {
    DISABLED,
    RETRACTING,
    MOVING_PIVOT,
    EXTENDING,
    FINISHED,
    CLIMBING,
    ESTOP
  };

  enum class ClimbState { PULLING_UP, PIVOTING_ROBOT, DONE };

  ArmState state_ = ArmState::DISABLED;
  ClimbState climb_state_ = ClimbState::DONE;
  RobotConstants constants;

  std::unique_ptr<Encoder> pivot_encoder_;
  std::unique_ptr<DigitalInput> pivot_hall_;
  std::unique_ptr<DoubleSolenoid> pivot_disk_brake_;
  std::unique_ptr<VictorSP> pivot_motor_a_, pivot_motor_b_;

  std::unique_ptr<Encoder> elevator_encoder_;
  std::unique_ptr<DoubleSolenoid> elevator_disk_brake_;
  std::unique_ptr<VictorSP> elevator_motor_a_, elevator_motor_b_;

  std::unique_ptr<Encoder> shooter_encoder_;
  std::unique_ptr<VictorSP> shooter_motor_a_, shooter_motor_b_;

  std::unique_ptr<Solenoid> shooter_hood_;

  std::unique_ptr<VictorSP> intake_front_;
  std::unique_ptr<VictorSP> intake_side_;

  std::unique_ptr<DigitalInput> ball_sensor_;

  PivotController pivot_controller_;
  ElevatorController elevator_controller_;
  ShooterBang shooter_controller_;

  muan::CSVLog csv_log_;
  SmartDashboardHelper csv_helper_;

  bool enabled_ = false;
  bool finished_ = true;
  bool climbing_done_ = false;
  bool climbing_advance_ = false;

  ArmGoal current_goal_;

  muan::Timer shot_timer_;
  bool should_shoot_ = false;
  const Time shot_time = 2 * s;

  Time t = 0 * s;

  IntakeGoal intake_target_;
  Time intake_timer_;

  Angle thresh_;

  void SetPivotBrake(bool on);
  void SetElevatorBrake(bool on);
  bool was_pivot_brake_{false}, was_elevator_brake_{false};
};

#endif /* ARM_ARM_SUBSYSTEM_H_ */
