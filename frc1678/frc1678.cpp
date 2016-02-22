#include <WPILib.h>
#include <memory>

#include "drivetrain/drivetrain_subsystem.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "CitrusButton.h"
#include "frc1678/auto/auto_routines.h"
#include "vision/vision.h"
#include "robot_subsystems.h"
#include "frc1678/frc1678.h"
#include "robot_constants/robot_constants.h"

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
  climb_pos_ = std::make_unique<CitrusButton>(j_manip_.get(), 4);
  climb_pos_continue_ = std::make_unique<CitrusButton>(j_manip_.get(), 7);
  climb_end_ = std::make_unique<CitrusButton>(j_manip_.get(), 8);
  wedge_toggle_ = std::make_unique<CitrusButton>(j_manip_.get(), 5);
  run_intake_until_ = std::make_unique<CitrusButton>(j_manip_.get(), 6);

  fender_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::SOUTH);
  long_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::NORTH);
  short_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::EAST);

  run_intake_forever_ = std::make_unique<CitrusAxis>(j_manip_.get(), 3);
  reverse_intake_ = std::make_unique<CitrusAxis>(j_manip_.get(), 2);

  // Auto
  auto_runner = new LemonScriptRunner("oneBall2016.auto", this);

  wedge_ = std::make_unique<Solenoid>(5);
}

void CitrusRobot::RobotInit() {
  subsystems_.drive.Start();
  subsystems_.arm.Start();
  std::cout << RobotConstants::GetInstance().pivot_calibration_offset.to(deg)
            << std::endl;
}

void CitrusRobot::AutonomousInit() {
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
}

void CitrusRobot::AutonomousPeriodic() { auto_runner->Update(); }

void CitrusRobot::TeleopInit() {
  // using muan::TrapezoidalMotionProfile;
  // subsystems_.drive.DriveDistance(2 * m);
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
  subsystems_.arm.GoToAutoShot();
}

void CitrusRobot::DisabledInit() {
  subsystems_.arm.SetEnabled(false);
  subsystems_.drive.SetEnabled(false);
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
    vision_.Start();
  }
  if (shift_high_->ButtonClicked()) {
    in_highgear_ = true;
  }
  if (shift_low_->ButtonClicked()) {
    in_highgear_ = false;
  }
  if (tuck_pos_->ButtonClicked()) {
    subsystems_.arm.GoToTuck();
    shootable_ = false;
    start_climb_ = false;
  }
  if (defensive_pos_->ButtonClicked()) {
    subsystems_.arm.GoToDefensive();
    shootable_ = false;
    start_climb_ = false;
  }
  if (intake_pos_->ButtonClicked()) {
    subsystems_.arm.GoToIntake();
    shootable_ = false;
    start_climb_ = false;
  }
  if (fender_pos_->ButtonClicked()) {
    subsystems_.arm.GoToFender();
    shootable_ = false;
    start_climb_ = false;
  }
  if (long_pos_->ButtonClicked()) {
    subsystems_.arm.GoToLong();
    shootable_ = true;
    start_climb_ = false;
  }
  if (run_intake_until_->ButtonPressed()) {
    subsystems_.arm.SetIntake(IntakeGoal::FORWARD_UNTIL);
  } else if (run_intake_forever_->ButtonClicked()) {
    subsystems_.arm.SetIntake(IntakeGoal::FORWARD_FOREVER);
  } else if (reverse_intake_->ButtonPressed()) {
    subsystems_.arm.SetIntake(IntakeGoal::REVERSE);
  } else if (run_intake_forever_->ButtonReleased() ||
             reverse_intake_->ButtonReleased()) {
    subsystems_.arm.SetIntake(IntakeGoal::OFF);
  }

  if (climb_pos_->ButtonClicked()) {
    subsystems_.arm.StartClimb();
    shootable_ = false;
    start_climb_ = true;
  }
  if (climb_pos_continue_->ButtonClicked()) {
    subsystems_.arm.ContinueClimb();
    shootable_ = false;
    start_climb_ = false;
  }
  if (climb_end_->ButtonClicked()) {
    subsystems_.arm.CompleteClimb();
    shootable_ = false;
    start_climb_ = false;
  }

  // Toggle the wedge when the button is deployed
  is_wedge_deployed_ = wedge_toggle_->ButtonClicked() ^ is_wedge_deployed_;
  if (wedge_toggle_->ButtonClicked()) {
    wedge_->Set(is_wedge_deployed_);
  }

  SetDriveGoal(&drivetrain_goal);
  subsystems_.drive.SetDriveGoal(drivetrain_goal);

  UpdateLights();
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
  climb_pos_->Update();
  climb_pos_continue_->Update();
  climb_end_->Update();
  intake_pos_->Update();
  fender_pos_->Update();
  long_pos_->Update();
  short_pos_->Update();
  run_intake_forever_->Update();
  run_intake_until_->Update();
  reverse_intake_->Update();
  wedge_toggle_->Update();
}

void CitrusRobot::UpdateLights() {
  //  if (subsystems_.arm.ClimbIsDone()) {
  //    lights_ = ColorLight::GREEN;
  //  }
  if (subsystems_.arm.AllIsDone() &&
      !vision_.IsSeeing()) {  // if arm is at position, not seeing
                              // vision
    lights_ = ColorLight::RED;
  }
  if (vision_.IsSeeing() &&
      subsystems_.arm
          .AllIsDone()) {  // if vision sees target + arm is done, yellow!
    lights_ = ColorLight::YELLOW;
  }
  if (vision_.Update(true) && shootable_ &&
      subsystems_.arm.AllIsDone()) {  // if aligned and ready to shoot
    lights_ = ColorLight::GREEN;
  }
  //  if (start_climb_ && subsystems_.arm.AllIsDone()) {
  //    lights_ = ColorLight::YELLOW;
  //  }
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
