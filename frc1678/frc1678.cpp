#include <WPILib.h>
#include <memory>

#include "robot_constants/robot_identifier.h"
#include "drivetrain/drivetrain_subsystem.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "utils/citrus_button.h"
#include "frc1678/auto/auto_routines.h"
#include "vision/robot/vision.h"
#include "robot_subsystems.h"
#include "frc1678/frc1678.h"
#include "robot_constants/robot_constants.h"
#include <iostream>
#include "muan/utils/timing_utils.h"

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
  cancel_profile_ = std::make_unique<CitrusButton>(j_stick_.get(), 8);
  proxy_shot_override_ = std::make_unique<CitrusButton>(j_manip_.get(), 9);
  recalibrate_pivot_ = std::make_unique<CitrusButton>(j_manip_.get(), 10);

  fender_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::SOUTH);
  long_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::NORTH);
  short_pos_ =
      std::make_unique<CitrusPOV>(j_manip_.get(), 0, POVPosition::EAST);

  run_intake_forever_ = std::make_unique<CitrusAxis>(j_manip_.get(), 3);
  reverse_intake_ = std::make_unique<CitrusAxis>(j_manip_.get(), 2);

  switch_one = std::make_unique<DigitalInput>(24);
  switch_two = std::make_unique<DigitalInput>(23);
  switch_three = std::make_unique<DigitalInput>(22);

  l_red_ = std::make_unique<DigitalOutput>(7);
  l_green_ = std::make_unique<DigitalOutput>(8);
  l_blue_ = std::make_unique<DigitalOutput>(9);

  disabled_ = true;

  wedge_ = std::make_unique<Solenoid>(5);

  auto_runner = nullptr;
}

void CitrusRobot::RobotInit() {
  subsystems_.drive.Start();
  subsystems_.arm.Start();
  SmartDashboard::PutString("Robot", GetRobotString(GetRobotIdentifier()));
  SmartDashboard::PutBoolean("Vision connection", (vision_.HasConnection()));
}

void CitrusRobot::AutonomousInit() {
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
  subsystems_.drive.gyro_reader_->SetOffset();

  subsystems_.arm.proxy_shot_override_ = true;
}

void CitrusRobot::AutonomousPeriodic() {
  auto_runner->Update();
  vision_.Update();
  wedge_->Set(is_wedge_deployed_);
}

void CitrusRobot::TeleopInit() {
  subsystems_.drive.SetEnabled(true);
  subsystems_.arm.SetEnabled(true);
  subsystems_.arm.proxy_shot_override_ = false;
  disabled_ = false;
  subsystems_.drive.CancelMotionProfile();
}

void CitrusRobot::DisabledInit() {
  subsystems_.arm.SetEnabled(false);
  subsystems_.drive.SetEnabled(false);
  disabled_ = true;
}

void CitrusRobot::DisabledPeriodic() {
  vision_.Update();
  UpdateAutoRoutine();
  UpdateLights();
  SmartDashboard::PutBoolean("Vision connection", (vision_.HasConnection()));
  j_manip_->SetRumble(Joystick::kRightRumble, 0);
  j_manip_->SetRumble(Joystick::kLeftRumble, 0);
}

void CitrusRobot::TeleopPeriodic() {
  // TODO (Finn): Get this out of the main loop and into its own
  // thread.
  vision_.Update();
  DrivetrainGoal drivetrain_goal;

  SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
  SmartDashboard::PutNumber("Stick", j_stick_->GetY());
  SmartDashboard::PutBoolean("Aligned", vision_.GetAligned());

  if (shoot_->ButtonClicked()) {
    subsystems_.arm.Shoot();
  }
  if (align_->ButtonPressed()) {
    // !button_was_pressed needs to be first due to short-circuiting logic
    if (!button_was_pressed_ && vision_.RunVision() && shootable_ && !subsystems_.arm.IsShooting()) {
      subsystems_.arm.Shoot(false);
      button_was_pressed_ = true;
    }
    was_running_vision_ = true;
  } else {
    if (was_running_vision_) {
      subsystems_.drive.CancelMotionProfile();
      was_running_vision_ = false;
    }
    button_was_pressed_ = false;
  }
  if (cancel_profile_->ButtonClicked()) {
    subsystems_.drive.CancelMotionProfile();
  }
  if (shift_high_->ButtonClicked()) {
    subsystems_.drive.Shift(true);
    drivetrain_goal.highgear = true;
  }
  if (shift_low_->ButtonClicked()) {
    subsystems_.drive.Shift(false);
    drivetrain_goal.highgear = false;
  }
  if (tuck_pos_->ButtonClicked()) {
    subsystems_.arm.GoToTuckSpin();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = true;
  }
  if (defensive_pos_->ButtonClicked()) {
    subsystems_.arm.GoToDefensiveSpin();
    shootable_ = false;
    start_climb_ = false;
    intaking_ = false;
    tuck_def_ = true;
  }
  if (intake_pos_->ButtonClicked()) {
    subsystems_.arm.GoToIntakeSpin();
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
  if(proxy_shot_override_->ButtonClicked()) {
    if(subsystems_.arm.proxy_shot_override_) {
      subsystems_.arm.proxy_shot_override_ = false;
    } else { subsystems_.arm.proxy_shot_override_ = true; }
  }
  if(recalibrate_pivot_->ButtonClicked()) {
    std::cout << "asdf"<<std::endl;
    subsystems_.arm.RecalibratePivot();
  }
  if(recalibrate_pivot_->ButtonReleased()) {
    std::cout << "hjkl"<<std::endl;
    subsystems_.arm.StopRecalibrating();
  }

  // Toggle the wedge when the button is deployed
  is_wedge_deployed_ = wedge_toggle_->ButtonClicked() ^ is_wedge_deployed_;
  wedge_->Set(is_wedge_deployed_);

  SmartDashboard::PutBoolean("Vision connection", (vision_.HasConnection()));

  SetDriveGoal(&drivetrain_goal);
  subsystems_.drive.SetDriveGoal(drivetrain_goal);

  UpdateLights();
  UpdateButtons();
}

void CitrusRobot::SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
  drivetrain_goal->steering = j_wheel_->GetX();
  drivetrain_goal->throttle = j_stick_->GetY();
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
  cancel_profile_->Update();
  proxy_shot_override_->Update();
  recalibrate_pivot_->Update();
}

void CitrusRobot::UpdateLights() {
  ColorLight light_color;

  // for enabled
  if (subsystems_.arm.AllIsDone() && !tuck_def_ &&
      !vision_.IsSeeing()) {  // if arm is at position, not seeing // vision
    light_color = ColorLight::RED;
  } else if (vision_.IsSeeing() && !tuck_def_ &&
             subsystems_.arm.AllIsDone()) {  // if vision sees target + arm is
                                             // done, yellow!
    light_color = ColorLight::YELLOW;
  }


  // Aligned and ready to shoot
  if (vision_.GetAligned() && shootable_ && !tuck_def_ &&
      subsystems_.arm.AllIsDone()  &&
      subsystems_.arm.ShooterSpeeded()) {
    light_color = ColorLight::GREEN;
  }

  if (!subsystems_.arm.AllIsDone()) {
    light_color = ColorLight::RED;
  }

  if (subsystems_.arm.AllIsDone() && tuck_def_) {
    light_color = ColorLight::GREEN;
  }

  if (!subsystems_.arm.BallIntaked() && intaking_) {
    light_color = ColorLight::BLUE;
    time = 0 * s;
  } else if (subsystems_.arm.BallIntaked() && intaking_) {
    light_color = ColorLight::GREEN;
    time += 0.02 * s;
    if (time < 1 * s && !disabled_) {
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

  if (disabled_) {
    j_manip_->SetRumble(Joystick::kRightRumble, 0);
    j_manip_->SetRumble(Joystick::kLeftRumble, 0);

    if (!subsystems_.drive.gyro_reader_->IsCalibrated()) {
      light_color = ColorLight::BLUE;
    } else {
      SmartDashboard::PutString("auto", auto_routine_);
      if (auto_routine_ == "one_ball.auto") {
        light_color = ColorLight::GREEN;
      } else if (auto_routine_ == "two_ball.auto") {
        light_color = ColorLight::RED;
      } else if (auto_routine_ == "class_d_left.auto") {
        light_color = FlashLights(ColorLight::YELLOW, ColorLight::GREEN);
      } else if (auto_routine_ == "class_d_right.auto") {
        light_color = FlashLights(ColorLight::PINK, ColorLight::GREEN);
      } else if (auto_routine_ == "class_a_left_right.auto") {
        light_color = FlashLights(ColorLight::YELLOW, ColorLight::BLUE);
      } else if (auto_routine_ == "class_a_right_right.auto") {
        light_color = FlashLights(ColorLight::PINK, ColorLight::BLUE);
      } else if (auto_routine_ == "class_a_left_left.auto") {
        light_color = FlashLights(ColorLight::YELLOW, ColorLight::RED);
      } else if (auto_routine_ == "class_a_right_left.auto") {
        light_color = FlashLights(ColorLight::PINK, ColorLight::RED);
      } else{
        light_color = ColorLight::WHITE;
      }
    }
  }

  light_color = FlashLights(light_color, light_color, !vision_.HasConnection());
  ColorLights(light_color);
}

ColorLight CitrusRobot::FlashLights(ColorLight color_one, ColorLight color_two, bool off_between) {
  auto color = (static_cast<int>(muan::now().to(s)) % 2) ? color_one : color_two;
  if (off_between && fmod(muan::now().to(s / 2), 0.5) < 0.25) color = ColorLight::OFF;
  return color;
}


void CitrusRobot::ColorLights(ColorLight value) {
  switch (value) {
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
    case ColorLight::TEAL:
      SetLightColor(0, 1, 1);
      break;
    case ColorLight::OFF:
      SetLightColor(0, 0, 0);
      break;
  }
}

void CitrusRobot::SetLightColor(int r, int g, int b) {
  l_red_->Set(r);
  l_green_->Set(g);
  l_blue_->Set(b);
}

void CitrusRobot::UpdateAutoRoutine() {
  std::map<int8_t, std::string> auto_map;

  auto_map[0b00000111] = "one_ball.auto";
  auto_map[0b00000110] = "two_ball.auto";
  auto_map[0b00000101] = "class_d_left.auto";
  auto_map[0b00000100] = "class_d_right.auto";
  auto_map[0b00000011] = "class_a_left_right.auto";
  auto_map[0b00000010] = "class_a_right_right.auto";
  auto_map[0b00000001] = "class_a_left_left.auto";
  auto_map[0b00000000] = "class_a_right_left.auto";

  int8_t auto_number = 0b00000000;
  auto_number |= (switch_one->Get() ? 0 : 1) << 0;
  auto_number |= (switch_two->Get() ? 0 : 1) << 1;
  auto_number |= (switch_three->Get() ? 0 : 1) << 2;

  if (auto_routine_ != auto_map[auto_number]) { // If the routine was just changed
    if (auto_runner != nullptr) {
      delete auto_runner;
    }
    auto_runner = new LemonScriptRunner("/home/lvuser/" + auto_map[auto_number], this);
  }

  auto_routine_ = auto_map[auto_number];
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
