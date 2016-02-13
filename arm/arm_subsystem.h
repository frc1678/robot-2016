#ifndef ARM_ARM_SUBSYSTEM_H_
#define ARM_ARM_SUBSYSTEM_H_

#include <WPILib.h>
#include <memory>
#include "muan/multithreading/updateable.h"
#include "pivot/pivot_controller.h"
#include "elevator/elevator_controller.h"
#include "shooter/shooter_bang.h"


struct ArmGoal {
  Angle pivot_goal;
  Length elevator_goal;
  AngularVelocity shooter_goal;
};

class ArmSubsystem : public muan::Updateable {
 public:
  ArmSubsystem();
  ~ArmSubsystem();

  void Update(Time dt) override;

  void GoToLong();
  void GoToTuck();
  void GoToFender();
  void GoToIntake();

  void SetHoodOpen(bool open);

  void SetEnabled(bool enabled);

  void SetIntake(bool on);
  void SetShooter(bool on);

 private:
 
  enum class ArmState {
    DISABLED, 
    RETRACTING,
    MOVING_PIVOT,
    EXTENDING, 
    FINISHED,
    ESTOP 
  };

  ArmState state_;

  void SetGoal(ArmGoal goal);

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

  PivotController pivot_controller_;
  ElevatorController elevator_controller_;
  ShooterBang shooter_controller_;

  bool enabled_ = false;

  ArmGoal current_goal_;
};

#endif /* ARM_ARM_SUBSYSTEM_H_ */
