#include <WPILib.h>
#include <memory>

#include "drivetrain/drivetrain_subsystem.h"
#include "frc1678/auto/auto_routines.h"
#include "frc1678/frc1678.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/utils/timing_utils.h"
#include "robot_constants/robot_constants.h"
#include "robot_constants/robot_identifier.h"
#include "robot_subsystems.h"
#include "utils/citrus_button.h"
#include "vision/robot/vision.h"
#include <iostream>

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

  lights_ = std::make_unique<LightController>();

  switch_one = std::make_unique<DigitalInput>(24);
  switch_two = std::make_unique<DigitalInput>(23);
  switch_three = std::make_unique<DigitalInput>(22);

  lights_->light_data_.disabled = true;

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
  lights_->light_data_.disabled = false;
  subsystems_.drive.CancelMotionProfile();
}

void CitrusRobot::DisabledInit() {
  subsystems_.arm.SetEnabled(false);
  subsystems_.drive.SetEnabled(false);
  lights_->light_data_.disabled = true;
}

void CitrusRobot::DisabledPeriodic() {
  vision_.Update();
  UpdateAutoRoutine();
  SmartDashboard::PutBoolean("Vision connection", (vision_.HasConnection()));

  {
    lights_->light_data_.arm_done = subsystems_.arm.AllIsDone();
    lights_->light_data_.vision_sees = vision_.IsSeeing();
    lights_->light_data_.vision_aligned = vision_.GetAligned();
    lights_->light_data_.vision_connected = vision_.HasConnection();
    lights_->light_data_.shooter_ready = subsystems_.arm.ShooterSpeeded();
    lights_->light_data_.gyro_calibrated =
        subsystems_.drive.gyro_reader_->IsCalibrated();
    lights_->light_data_.ball_intaked = subsystems_.arm.BallIntaked();
  }

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
    if (!button_was_pressed_ && vision_.RunVision() && shootable_ &&
        !subsystems_.arm.IsShooting()) {
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
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = true;
  }
  if (defensive_pos_->ButtonClicked()) {
    subsystems_.arm.GoToDefensiveSpin();
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = true;
  }
  if (intake_pos_->ButtonClicked()) {
    subsystems_.arm.GoToIntakeSpin();
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = true;
    lights_->light_data_.tuck_def_pos = false;
  }
  if (fender_pos_->ButtonClicked()) {
    subsystems_.arm.GoToFender();
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = false;
  }
  if (long_pos_->ButtonClicked()) {
    subsystems_.arm.GoToLong();
    lights_->light_data_.shoot_pos = true;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = false;
  }
  if (short_pos_->ButtonClicked()) {
    subsystems_.arm.GoToAutoShot();
    lights_->light_data_.shoot_pos = true;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = false;
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
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = false;
  }
  if (climb_pos_continue_->ButtonClicked()) {
    subsystems_.arm.ContinueClimb();
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = false;
  }
  if (climb_end_->ButtonClicked()) {
    subsystems_.arm.CompleteClimb();
    lights_->light_data_.shoot_pos = false;
    lights_->light_data_.intake_pos = false;
    lights_->light_data_.tuck_def_pos = false;
  }
  if (proxy_shot_override_->ButtonClicked()) {
    if (subsystems_.arm.proxy_shot_override_) {
      subsystems_.arm.proxy_shot_override_ = false;
    } else {
      subsystems_.arm.proxy_shot_override_ = true;
    }
  }
  if (recalibrate_pivot_->ButtonClicked()) {
    std::cout << "asdf" << std::endl;
    subsystems_.arm.RecalibratePivot();
  }
  if (recalibrate_pivot_->ButtonReleased()) {
    std::cout << "hjkl" << std::endl;
    subsystems_.arm.StopRecalibrating();
  }

  {
    lights_->light_data_.arm_done = subsystems_.arm.AllIsDone();
    lights_->light_data_.vision_sees = vision_.IsSeeing();
    lights_->light_data_.vision_aligned = vision_.GetAligned();
    lights_->light_data_.vision_connected = vision_.HasConnection();
    lights_->light_data_.shooter_ready = subsystems_.arm.ShooterSpeeded();
    lights_->light_data_.gyro_calibrated =
        subsystems_.drive.gyro_reader_->IsCalibrated();
    lights_->light_data_.ball_intaked = subsystems_.arm.BallIntaked();
  }

  // Toggle the wedge when the button is deployed
  is_wedge_deployed_ = wedge_toggle_->ButtonClicked() ^ is_wedge_deployed_;
  wedge_->Set(is_wedge_deployed_);

  SmartDashboard::PutBoolean("Vision connection", (vision_.HasConnection()));

  SetDriveGoal(&drivetrain_goal);
  subsystems_.drive.SetDriveGoal(drivetrain_goal);

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

  if (auto_routine_ !=
      auto_map[auto_number]) {  // If the routine was just changed
    if (auto_runner != nullptr) {
      delete auto_runner;
    }
    auto_runner =
        new LemonScriptRunner("/home/lvuser/" + auto_map[auto_number], this);
  }

  auto_routine_ = auto_map[auto_number];
  lights_->light_data_.auto_selection = auto_routine_;
}

CitrusRobot::~CitrusRobot() {}

START_ROBOT_CLASS(CitrusRobot);
