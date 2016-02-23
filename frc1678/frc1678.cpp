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
  climb_pos_ = std::make_unique<CitrusButton>(j_manip_.get(), 4);
  climb_pos_continue_ = std::make_unique<CitrusButton>(j_manip_.get(), 7);
  climb_end_ = std::make_unique<CitrusButton>(j_manip_.get(), 8);

  fender_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::SOUTH);
  long_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::NORTH);

  run_intake_ = std::make_unique<CitrusAxis>(j_manip_.get(), 3);
  reverse_intake_ = std::make_unique<CitrusAxis>(j_manip_.get(), 2);

  // Auto
  auto_runner = new LemonScriptRunner("twoBall2016.auto", this);

  l_pow_ = std::make_unique<DigitalOutput>(25);
  l_red_ = std::make_unique<DigitalOutput>(7);
  l_green_ = std::make_unique<DigitalOutput>(8);
  l_blue_ = std::make_unique<DigitalOutput>(9);

  disabled_ = true;
}

void CitrusRobot::RobotInit() {
  subsystems_.drive.Start();
  subsystems_.arm.Start();
}

void CitrusRobot::AutonomousInit() {}

void CitrusRobot::AutonomousPeriodic() { auto_runner->Update(); }

void CitrusRobot::TeleopInit() {
  // using muan::TrapezoidalMotionProfile;
  // subsystems_.drive.DriveDistance(2 * m);
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
  disabled_ = false;
}

void CitrusRobot::DisabledInit() {
  subsystems_.arm.SetEnabled(false);
  subsystems_.drive.SetEnabled(false);
  disabled_ = true;
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
  UpdateLights();
  ColorLights();
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
  if (run_intake_->ButtonPressed()) {
    subsystems_.arm.SetIntake(IntakeGoal::FORWARD);
  } else if (reverse_intake_->ButtonPressed()) {
    subsystems_.arm.SetIntake(IntakeGoal::REVERSE);
  } else {
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

  SetDriveGoal(&drivetrain_goal);
  subsystems_.drive.SetDriveGoal(drivetrain_goal);

  UpdateLights();
  ColorLights();
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
  run_intake_->Update();
  reverse_intake_->Update();
}

void CitrusRobot::UpdateLights() {
  if (!subsystems_.drive.gyro_reader_->IsCalibrated()) {
    lights_ = ColorLight::BLUE;
  } else if (disabled_) {
    lights_ = ColorLight::WHITE;
  } else if (subsystems_.arm.AllIsDone() &&
             !vision_.IsSeeing()) {  // if arm is at position, not seeing
    // vision
    lights_ = ColorLight::RED;
  } else if (vision_.IsSeeing() &&
             subsystems_.arm.AllIsDone()) {  // if vision sees target + arm is
                                             // done, yellow!
    lights_ = ColorLight::YELLOW;
  } else if (vision_.Update(true) && shootable_ &&
             subsystems_.arm.AllIsDone()) {  // if aligned and ready to shoot
    lights_ = ColorLight::GREEN;
  } else {
    lights_ = ColorLight::PINK;
  }

  // if (start_climb_ && subsystems_.arm.AllIsDone()) {
  //  lights_ = ColorLight::YELLOW;
  //}
  // if (subsystems_.arm.ClimbIsDone()) {
  //  lights_ = ColorLight::GREEN;
  //}
}

void CitrusRobot::ColorLights() {
  switch (lights_) {
    case ColorLight::RED:
      l_red_->Set(1);
      l_green_->Set(0);
      l_blue_->Set(0);
      break;
    case ColorLight::YELLOW:
      l_red_->Set(1);
      l_green_->Set(1);
      l_blue_->Set(0);
      break;
    case ColorLight::GREEN:
      l_red_->Set(0);
      l_green_->Set(1);
      l_blue_->Set(0);
      break;
    case ColorLight::BLUE:
      l_red_->Set(0);
      l_green_->Set(0);
      l_blue_->Set(1);
      break;
    case ColorLight::WHITE:
      l_red_->Set(1);
      l_green_->Set(1);
      l_blue_->Set(1);
      break;
    case ColorLight::PINK:
      l_red_->Set(1);
      l_green_->Set(0);
      l_blue_->Set(1);
      break;
  }
  l_pow_->Set(1);
  std::cout << "lights" << static_cast<int>(lights_) << std::endl;
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
