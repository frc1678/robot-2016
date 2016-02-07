#include <WPILib.h>
#include <memory>

#include "drivetrain/drivetrain_subsystem.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "CitrusButton.h"
#include "frc1678/auto/auto_routines.h"
#include "vision/vision.h"
#include "robot_subsystems.h"
#include "frc1678/frc1678.h"

CitrusRobot::CitrusRobot() : vision_(subsystems_) {
  // Joysticks
  j_wheel_ = std::make_unique<Joystick>(0);
  j_stick_ = std::make_unique<Joystick>(1);
  // manipulator = new Joystick(2);

  // Buttonz!
  shift_down_ = std::make_unique<CitrusButton>(j_stick_.get(), 2);
  shift_up_ = std::make_unique<CitrusButton>(j_stick_.get(), 1);
  quick_turn_ = std::make_unique<CitrusButton>(j_wheel_.get(), 5);
    
  // Auto
  auto_runner = new LemonScriptRunner("test_align.auto", this);
}

void CitrusRobot::RobotInit() { subsystems_.drive.Start(); }

void CitrusRobot::AutonomousInit() {
  
}

void CitrusRobot::AutonomousPeriodic() { 
  auto_runner->Update();
  //if (!vision_done_) {
  //  vision_done_ = vision_.Update();
  //}
}

void CitrusRobot::TeleopInit() {
  using muan::TrapezoidalMotionProfile;
  auto dp = std::make_unique<TrapezoidalMotionProfile<Length>>(
      0 * m, 5 * ft / s, 10 * ft / s / s);
  auto ap = std::make_unique<TrapezoidalMotionProfile<Angle>>(
      -20 * deg, 240 * deg / s, 500 * deg / s / s);
  subsystems_.drive.FollowMotionProfile(std::move(dp), std::move(ap));
}

void CitrusRobot::DisabledPeriodic() {
  // TODO (Finn): Get this out of the main loop and into its own
  // thread.
  DrivetrainGoal drivetrain_goal;

  if (test_flag_) {
    vision_.EndTest();
    test_flag_ = false;
  }

  // SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
  // SmartDashboard::PutNumber("Stick", j_stick_->GetY());
  SetDriveGoal(&drivetrain_goal);
  vision_done_ = vision_.Update(false);

  subsystems_.drive.SetDriveGoal(drivetrain_goal);
}

void CitrusRobot::TeleopPeriodic() {
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

  UpdateButtons();
}

void CitrusRobot::SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
  drivetrain_goal->steering = j_wheel_->GetX();
  drivetrain_goal->throttle = j_stick_->GetY();
  drivetrain_goal->highgear = in_highgear_;
  drivetrain_goal->quickturn = quick_turn_->ButtonPressed();
  drivetrain_goal->control_loop_driving = false;
}

void CitrusRobot::UpdateButtons() {
  shift_down_->Update();
  shift_up_->Update();
  quick_turn_->Update();
}

CitrusRobot::~CitrusRobot() {}


START_ROBOT_CLASS(CitrusRobot);
