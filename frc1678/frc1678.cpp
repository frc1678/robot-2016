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
  j_manip_ = std::make_unique<Joystick>(2);

  // Buttonz!
  shoot_ = std::make_unique<CitrusButton>(j_stick_.get(), 2);
  align_ = std::make_unique<CitrusButton>(j_stick_.get(), 1);
  shift_high_ = std::make_unique<CitrusButton>(j_stick_.get(), 5);
  shift_low_ = std::make_unique<CitrusButton>(j_stick_.get(), 4);
  quick_turn_ = std::make_unique<CitrusButton>(j_wheel_.get(), 5);

  tuck_pos_ = std::make_unique<CitrusButton>(j_manip_.get(), 1);
  defensive_pos_ = std::make_unique<CitrusButton>(j_manip_.get(), 2);
  intake_pos_ = std::make_unique<CitrusButton>(j_manip_.get(), 3);
  climb_ = std::make_unique<CitrusButton>(j_manip_.get(), 4);

  fender_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::SOUTH);
  long_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::NORTH);

  run_intake_ = std::make_unique<CitrusAxis>(j_manip_.get(), 3);
  reverse_intake_ = std::make_unique<CitrusAxis>(j_manip_.get(), 2);

  // Auto
  auto_runner = new LemonScriptRunner("oneBall2016.auto", this);
}

void CitrusRobot::RobotInit() {
  subsystems_.drive.Start();
  subsystems_.arm.Start();
  subsystems_.arm.SetEnabled(true);
}

void CitrusRobot::AutonomousInit() {}

void CitrusRobot::AutonomousPeriodic() { auto_runner->Update(); }

void CitrusRobot::TeleopInit() {
  using muan::TrapezoidalMotionProfile;
//  subsystems_.arm.SetEnabled(true);
  subsystems_.drive.DriveDistance(3 * ft);
}

void CitrusRobot::DisabledPeriodic() {
  // TODO (Finn): Get this out of the main loop and into its own
  // thread.
  // DrivetrainGoal drivetrain_goal;

  // if (test_flag_) {
  // vision_.EndTest();
  // test_flag_ = false;
  // }

  // SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
  // SmartDashboard::PutNumber("Stick", j_stick_->GetY());
  //  SetDriveGoal(&drivetrain_goal);
  //  vision_done_ = vision_.Update(false);

  //  subsystems_.drive.SetDriveGoal(drivetrain_goal);
}

void CitrusRobot::TeleopPeriodic() {
  // TODO (Finn): Get this out of the main loop and into its own
  // thread.
  DrivetrainGoal drivetrain_goal;

  SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
  SmartDashboard::PutNumber("Stick", j_stick_->GetY());

  if (shoot_->ButtonClicked()) {
    subsystems_.arm.Shoot();
  }
  if (align_->ButtonClicked()) {
  }
  if (shift_high_->ButtonClicked()) {
    in_highgear_ = true;
  }
  if (shift_low_->ButtonClicked()) {
    in_highgear_ = false;
  }
  if (tuck_pos_->ButtonClicked()) {
    subsystems_.arm.GoToTuck();
  }
  if (defensive_pos_->ButtonClicked()) {
    subsystems_.arm.GoToDefensive();
  }
  if (intake_pos_->ButtonClicked()) {
    subsystems_.arm.GoToIntake();
  }
  if (fender_pos_->ButtonClicked()) {
    subsystems_.arm.GoToFender();
  }
  if (long_pos_->ButtonClicked()) {
    subsystems_.arm.GoToLong();
  }
  if (run_intake_->ButtonPressed()) {
    subsystems_.arm.SetIntake(IntakeGoal::FORWARD);
  } else if (reverse_intake_->ButtonPressed()) {
    subsystems_.arm.SetIntake(IntakeGoal::REVERSE);
  } else {
    subsystems_.arm.SetIntake(IntakeGoal::OFF);
  }

  SetDriveGoal(&drivetrain_goal);
  subsystems_.drive.SetDriveGoal(drivetrain_goal);

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
  shoot_->Update();
  align_->Update();
  shift_high_->Update();
  shift_low_->Update();
  quick_turn_->Update();
  tuck_pos_->Update();
  defensive_pos_->Update();
  climb_->Update();
  intake_pos_->Update();
  fender_pos_->Update();
  long_pos_->Update();
  run_intake_->Update();
  reverse_intake_->Update();
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
