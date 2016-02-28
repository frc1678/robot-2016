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

CitrusRobot::CitrusRobot()
    : vision_(subsystems_, RobotConstants::GetInstance()) {
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
  auto_runner = new LemonScriptRunner("classD.auto", this);

  l_pow_ = std::make_unique<DigitalOutput>(25);
  l_red_ = std::make_unique<DigitalOutput>(7);
  l_green_ = std::make_unique<DigitalOutput>(8);
  l_blue_ = std::make_unique<DigitalOutput>(9);

  disabled_ = true;

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
  subsystems_.drive.gyro_reader_->SetOffset(
      subsystems_.drive.gyro_reader_->GetAngle());
}

void CitrusRobot::AutonomousPeriodic() {
  auto_runner->Update();
  printf("wedgetf? %d\n", is_wedge_deployed_);
  wedge_->Set(is_wedge_deployed_);
}

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
    intaking_ = false;
    tuck_def_ = true;
  }
  if (defensive_pos_->ButtonClicked()) {
    subsystems_.arm.GoToDefensive();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = true;
  }
  if (intake_pos_->ButtonClicked()) {
    subsystems_.arm.GoToIntake();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = true;
    tuck_def_ = false;
    time = 0 * s;
  }
  if (fender_pos_->ButtonClicked()) {
    subsystems_.arm.GoToFender();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = false;
  }
  if (long_pos_->ButtonClicked()) {
    subsystems_.arm.GoToLong();
    shootable_ = true;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = false;
  }
  if (short_pos_->ButtonClicked()) {
    subsystems_.arm.GoToAutoShot();
    shootable_ = true;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = false;
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
    intaking_ = false;
    tuck_def_ = false;
  }
  if (climb_pos_continue_->ButtonClicked()) {
    subsystems_.arm.ContinueClimb();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = false;
  }
  if (climb_end_->ButtonClicked()) {
    subsystems_.arm.CompleteClimb();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = false;
  }

  // Toggle the wedge when the button is deployed
  is_wedge_deployed_ = wedge_toggle_->ButtonClicked() ^ is_wedge_deployed_;
  // if (wedge_toggle_->ButtonClicked()) {
  wedge_->Set(is_wedge_deployed_);
  //}

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
  short_pos_->Update();
  run_intake_forever_->Update();
  run_intake_until_->Update();
  reverse_intake_->Update();
  wedge_toggle_->Update();
}

void CitrusRobot::UpdateLights() {
  // for enabled
  if (subsystems_.arm.AllIsDone() && !tuck_def_ &&
      !vision_.IsSeeing()) {  // if arm is at position, not seeing
    // vision
    lights_ = ColorLight::RED;
  } else if (vision_.IsSeeing() && !tuck_def_ &&
             subsystems_.arm.AllIsDone()) {  // if vision sees target + arm is
                                             // done, yellow!
    lights_ = ColorLight::YELLOW;
  }

  if (vision_.Update(true) && shootable_ && !tuck_def_ &&
      subsystems_.arm.AllIsDone() &&
      subsystems_.arm.ShooterSpeeded()) {  // if aligned and ready to shoot
    lights_ = ColorLight::GREEN;
  }

  if (!subsystems_.arm.AllIsDone()) {
    lights_ = ColorLight::RED;
  }

  if (subsystems_.arm.AllIsDone() && tuck_def_) {
    lights_ = ColorLight::GREEN;
  }
  // for intaking
  if (!subsystems_.arm.BallIntaked() && intaking_) {
    lights_ = ColorLight::BLUE;
    time = 0 * s;
  } else if (subsystems_.arm.BallIntaked() && intaking_) {
    lights_ = ColorLight::GREEN;
    time += 0.02 * s;
    if (time < 1.5 * s) {
      j_manip_->SetRumble(Joystick::kLeftRumble, 1);
      j_manip_->SetRumble(Joystick::kRightRumble, 1);
    } else {
      j_manip_->SetRumble(Joystick::kLeftRumble, 0);
      j_manip_->SetRumble(Joystick::kRightRumble, 0);
    }
  }

  // for disabled
  if (disabled_ && !subsystems_.drive.gyro_reader_->IsCalibrated()) {
    lights_ = ColorLight::BLUE;
  } else if (disabled_) {
    lights_ = ColorLight::WHITE;
  }

  // if (start_climb_ && subsystems_.arm.AllIsDone()) {
  //  lights_ = ColorLight::YELLOW;
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
  //  if (start_climb_ && subsystems_.arm.AllIsDone()) {
  //    lights_ = ColorLight::YELLOW;
  //  }
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
