#include <WPILib.h>
#include <memory>

#include "robot_constants/robot_identifier.h"
#include "drivetrain/drivetrain_subsystem.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "utils/citrus_button.h"
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

  switch_one = new DigitalInput(23);
  switch_two = new DigitalInput(24);

  l_pow_ = std::make_unique<DigitalOutput>(25);
  l_red_ = std::make_unique<DigitalOutput>(7);
  l_green_ = std::make_unique<DigitalOutput>(8);
  l_blue_ = std::make_unique<DigitalOutput>(9);

  disabled_ = true;

  wedge_ = std::make_unique<Solenoid>(5);

  auto_runner = nullptr;
}

std::string CitrusRobot::GetAutoRoutine() {
  std::map<int8_t, std::string> auto_map;

  auto_map[0b00000011] = "one_ball.auto";
  auto_map[0b00000001] = "one_ball.auto";
  auto_map[0b00000010] = "class_d_left.auto";
  auto_map[0b00000000] = "class_d_right.auto";

  int8_t auto_number = 0b00000000;
  auto_number |= (switch_one->Get() ? 0 : 1) << 0;
  auto_number |= (switch_two->Get() ? 0 : 1) << 1;

  return auto_map[auto_number];
}

void CitrusRobot::RobotInit() {
  subsystems_.drive.Start();
  subsystems_.arm.Start();
  std::vector<std::string> robot_names = {"comp", "appa", "ssbb", "wtf"};
  SmartDashboard::PutString("Robot", robot_names[(int)GetRobotIdentifier()]);
}

void CitrusRobot::AutonomousInit() {
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
  subsystems_.drive.gyro_reader_->SetOffset();

  if (auto_runner != nullptr) {
    delete auto_runner;
  }
  auto_runner = new LemonScriptRunner("/home/lvuser/" + GetAutoRoutine(), this);
}

void CitrusRobot::AutonomousPeriodic() {
  auto_runner->Update();
  wedge_->Set(is_wedge_deployed_);
}

void CitrusRobot::TeleopInit() {
  // using muan::TrapezoidalMotionProfile;
  // subsystems_.drive.DriveDistance(2 * m);
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
  disabled_ = false;
  subsystems_.drive.CancelMotionProfile();
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
  j_manip_->SetRumble(Joystick::kRightRumble, 0);
  j_manip_->SetRumble(Joystick::kLeftRumble, 0);
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

  if (vision_.Aligned() && shootable_ && !tuck_def_ &&
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
    time += 0.05 * s;
    if (time < 1.5 * s && !disabled_) {
      j_manip_->SetRumble(Joystick::kLeftRumble, 1);
      j_manip_->SetRumble(Joystick::kRightRumble, 1);
    } else {
      j_manip_->SetRumble(Joystick::kLeftRumble, 0);
      j_manip_->SetRumble(Joystick::kRightRumble, 0);
    }
  } else {
    time = 0 * s;
    j_manip_->SetRumble(Joystick::kLeftRumble, 0);
    j_manip_->SetRumble(Joystick::kRightRumble, 0);
  }

  // for disabled
  if (disabled_ && !subsystems_.drive.gyro_reader_->IsCalibrated()) {
    lights_ = ColorLight::BLUE;
  } else if (disabled_) {
    j_manip_->SetRumble(Joystick::kRightRumble, 0);
    j_manip_->SetRumble(Joystick::kLeftRumble, 0);

    std::string auto_routine = GetAutoRoutine();
    if (auto_routine == "one_ball.auto") {
      lights_ = ColorLight::GREEN;
    } else if (auto_routine == "two_ball.auto") {
      lights_ = ColorLight::RED;
    } else if (auto_routine == "class_d_left.auto") {
      lights_ = ColorLight::YELLOW;
    } else if (auto_routine == "class_d_right.auto") {
      lights_ = ColorLight::PINK;
    } else {
      lights_ = ColorLight::WHITE;
    }
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
      SetLightColor(1, 0, 0);
      break;
    case ColorLight::YELLOW:
      SetLightColor(1, 1, 0);
      break;
    case ColorLight::GREEN:
      SetLightColor(0, 1, 0);
      break;
    case ColorLight::BLUE:
      SetLightColor(0, 0, 1);
      break;
    case ColorLight::WHITE:
      SetLightColor(1, 1, 1);
      break;
    case ColorLight::PINK:
      SetLightColor(1, 0, 1);
      break;
  }
}

void CitrusRobot::SetLightColor(int r, int g, int b) {
  l_red_->Set(r);
  l_green_->Set(g);
  l_blue_->Set(b);
  l_pow_->Set(1);
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
