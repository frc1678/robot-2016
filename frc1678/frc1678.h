#ifndef FRC1678_H
#define FRC1678_H

#include "frc1678/auto/auto_routines.h"
#include "frc1678/lights.h"
#include "robot_subsystems.h"
#include "vision/robot/vision.h"
// class LemonScriptRunner { };

class CitrusButton;
class CitrusAxis;
class CitrusPOV;
class LightController;

class CitrusRobot : public IterativeRobot {
 private:
  LemonScriptRunner* auto_runner;
  std::string auto_routine_;
  std::unique_ptr<Solenoid> wedge_;

  std::unique_ptr<DigitalInput> switch_one, switch_two, switch_three;

  std::unique_ptr<Joystick> j_wheel_, j_stick_, j_manip_;

  std::unique_ptr<LightController> lights_;

  // Avery's buttons
  std::unique_ptr<CitrusButton> shoot_, align_, shift_high_, shift_low_,
      quick_turn_;

  // Kelly's buttons
  std::unique_ptr<CitrusButton> tuck_pos_, defensive_pos_, climb_pos_,
      climb_pos_continue_, climb_end_, intake_pos_, wedge_toggle_,
      run_intake_until_, cancel_profile_, proxy_shot_override_,
      recalibrate_pivot_;
  std::unique_ptr<CitrusPOV> fender_pos_, long_pos_, short_pos_;
  std::unique_ptr<CitrusAxis> run_intake_forever_, reverse_intake_;

 public:
  RobotSubsystems subsystems_;
  CitrusVision vision_;

  Timer* camera_timer_ = new Timer();

  bool is_wedge_deployed_ = false;
  bool in_highgear_;
  bool shootable_ = false;
  bool intaking_ = false;
  bool tuck_def_ = false;
  bool disabled_;
  bool was_running_vision_ = false;
  bool button_was_pressed_ = false;
  Time time = 0 * s;

  CitrusRobot();
  void RobotInit();
  void UpdateAutoRoutine();
  void AutonomousInit();
  void AutonomousPeriodic();
  void TeleopInit();
  void DisabledInit();
  void DisabledPeriodic();
  void TeleopPeriodic();
  void SetDriveGoal(DrivetrainGoal* drivetrain_goal);

  void UpdateButtons();
  ~CitrusRobot();
};

#endif
