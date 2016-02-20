#ifndef FRC1678_H
#define FRC1678_H

#include "CitrusButton.h"
#include "frc1678/auto/auto_routines.h"
#include "vision/vision.h"
#include "robot_subsystems.h"

// class LemonScriptRunner { };

enum class ColorLight { RED = 0, YELLOW, GREEN };

class CitrusRobot : public IterativeRobot {
 private:
  LemonScriptRunner* auto_runner;

 public:
  std::unique_ptr<Joystick> j_wheel_, j_stick_, j_manip_;

  RobotSubsystems subsystems_;
  CitrusVision vision_;

  // Avery's buttons
  std::unique_ptr<CitrusButton> shoot_, align_, shift_high_, shift_low_,
      quick_turn_;

  // Kelly's buttons
  std::unique_ptr<CitrusButton> tuck_pos_, defensive_pos_, climb_pos_,
      climb_pos_continue_, climb_end_, intake_pos_;
  std::unique_ptr<CitrusPOV> fender_pos_, long_pos_;
  std::unique_ptr<CitrusAxis> run_intake_, reverse_intake_;

  ColorLight lights_;

  bool test_flag_;
  bool in_highgear_;
  bool vision_done_ = false;  // UGLY HACK
  bool shootable_ = false;    // ALSO UGLY HACK
  bool start_climb_ = false;  // ANOTHER UGLY HACK

  CitrusRobot();
  void RobotInit();
  void AutonomousInit();
  void AutonomousPeriodic();
  void TeleopInit();
  void DisabledInit();
  void DisabledPeriodic();
  void TeleopPeriodic();
  void SetDriveGoal(DrivetrainGoal* drivetrain_goal);
  void UpdateLights();
  void UpdateButtons();
  ~CitrusRobot();
};

#endif
