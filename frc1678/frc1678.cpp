#include <WPILib.h>
#include <memory>

#include "drivetrain/drivetrain_subsystem.h"
#include "CitrusButton.h"
#include "frc1678/auto/auto_routines.h"
#include "robot_subsystems.h"

class CitrusRobot : public IterativeRobot {
 private:
  LemonScriptRunner* auto_runner;

 public:
  std::unique_ptr<Joystick> j_wheel_, j_stick_;

  RobotSubsystems subsystems_;

  // Buttonz!
  std::unique_ptr<CitrusButton> shift_down_, shift_up_, quick_turn_;

  bool in_highgear_;

  CitrusRobot() {
    // Joysticks
    j_wheel_ = std::make_unique<Joystick>(0);
    j_stick_ = std::make_unique<Joystick>(1);
    // manipulator = new Joystick(2);

    // Buttonz!
    shift_down_ = std::make_unique<CitrusButton>(j_stick_.get(), 2);
    shift_up_ = std::make_unique<CitrusButton>(j_stick_.get(), 1);
    quick_turn_ = std::make_unique<CitrusButton>(j_wheel_.get(), 5);
    
    // Auto
    auto_runner = new LemonScriptRunner("twoBall2016.auto", &subsystems_);
  }

  void RobotInit() { subsystems_.drive.Start(); }

  void AutonomousInit() {}

  void AutonomousPeriodic() { 
          auto_runner->Update(); }

  void TeleopInit() {}

  void DisabledPeriodic() {
    // TODO (Finn): Get this out of the main loop and into its own
    // thread.
    DrivetrainGoal drivetrain_goal;

    SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
    SmartDashboard::PutNumber("Stick", j_stick_->GetY());
    SetDriveGoal(&drivetrain_goal);

    subsystems_.drive.SetDriveGoal(drivetrain_goal);
  }

  void TeleopPeriodic() {
    // TODO (Finn): Get this out of the main loop and into its own
    // thread.
    DrivetrainGoal drivetrain_goal;

    SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
    SmartDashboard::PutNumber("Stick", j_stick_->GetY());

    // TODO (Finn): Act on the output, without bypassing the
    // controller. Or argue that this is fine.
    if (shift_up_->ButtonClicked()) {
      in_highgear_ = true;
    } else if (shift_down_->ButtonClicked()) {
      in_highgear_ = false;
    }

    SetDriveGoal(&drivetrain_goal);

    subsystems_.drive.SetDriveGoal(drivetrain_goal);

    UpdateButtons();
  }

  void SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
    drivetrain_goal->steering = j_wheel_->GetX();
    drivetrain_goal->throttle = j_stick_->GetY();
    drivetrain_goal->highgear = in_highgear_;
    drivetrain_goal->quickturn = quick_turn_->ButtonPressed();
    drivetrain_goal->control_loop_driving = false;
  }

  void UpdateButtons() {
    shift_down_->Update();
    shift_up_->Update();
    quick_turn_->Update();
  }

  ~CitrusRobot() {}
};

START_ROBOT_CLASS(CitrusRobot);
