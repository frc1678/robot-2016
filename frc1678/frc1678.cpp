#include <WPILib.h>
#include <memory>

#include "drivetrain/drivetrain_subsystem.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "CitrusButton.h"
#include "vision/vision.h"
class CitrusRobot : public IterativeRobot {
  std::unique_ptr<Joystick> j_wheel_, j_stick_;

  std::unique_ptr<DrivetrainSubsystem> drive_subsystem_;

  // Buttonz!
  std::unique_ptr<CitrusButton> shift_down_, shift_up_, quick_turn_;

  bool in_highgear_;

 public:
  CitrusRobot() {
    // Joysticks
    j_wheel_ = std::make_unique<Joystick>(0);
    j_stick_ = std::make_unique<Joystick>(1);
    // manipulator = new Joystick(2);

    // Buttonz!
    shift_down_ = std::make_unique<CitrusButton>(j_stick_.get(), 2);
    shift_up_ = std::make_unique<CitrusButton>(j_stick_.get(), 1);
    quick_turn_ = std::make_unique<CitrusButton>(j_wheel_.get(), 5);

    drive_subsystem_ = std::make_unique<DrivetrainSubsystem>();
  }

  void RobotInit() { drive_subsystem_->Start(); }

  void TeleopInit() {
    using muan::TrapezoidalMotionProfile;
    auto dp = std::make_unique<TrapezoidalMotionProfile<Length>>(0*m, 5*ft/s, 10*ft/s/s);
    auto ap = std::make_unique<TrapezoidalMotionProfile<Angle>>(90*deg, 0.27*1.5*rev/s, 270*deg/s/s);
    drive_subsystem_->FollowMotionProfile(std::move(dp), std::move(ap));
  }

  void AutonomousInit() {}
  void AutonomousPeriodic() {
          drive_subsystem_->SetDriveGoal(runAlignment());
  }
  void DisabledPeriodic() {
    // TODO (Finn): Get this out of the main loop and into its own
    // thread.
    //DrivetrainGoal drivetrain_goal;

    //SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
    //SmartDashboard::PutNumber("Stick", j_stick_->GetY());
    //SetDriveGoal(&drivetrain_goal);

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

    //SetDriveGoal(&drivetrain_goal);

    //drive_subsystem_->SetDriveGoal(drivetrain_goal);

    //drive_subsystem_->SetDriveGoal(drivetrain_goal);

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
