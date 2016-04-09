#ifndef FRC1678_H
#define FRC1678_H

#include "frc1678/auto/auto_routines.h"
#include "vision/vision.h"
#include "robot_subsystems.h"

// class LemonScriptRunner { };

enum class ColorLight { RED = 0, YELLOW, GREEN, BLUE, WHITE, PINK, OFF };

class CitrusButton;
class CitrusAxis;
class CitrusPOV;

class CitrusRobot : public IterativeRobot {
 private:
  LemonScriptRunner* auto_runner;
  std::unique_ptr<Solenoid> wedge_;

  // Avery's buttons
  std::unique_ptr<CitrusButton> shoot_, align_, shift_high_, shift_low_,
      quick_turn_;

  // Kelly's buttons
  std::unique_ptr<CitrusButton> tuck_pos_, defensive_pos_, climb_pos_,
      climb_pos_continue_, climb_end_, intake_pos_, wedge_toggle_,
      run_intake_until_, cancel_profile_, proxy_shot_override_;
  std::unique_ptr<CitrusPOV> fender_pos_, long_pos_, short_pos_;
  std::unique_ptr<CitrusAxis> run_intake_forever_, reverse_intake_;


 public:
  std::unique_ptr<Joystick> j_wheel_, j_stick_, j_manip_;

  RobotSubsystems subsystems_;
  CitrusVision vision_;

  ColorLight lights_;
  std::unique_ptr<DigitalOutput> l_pow_, l_red_, l_green_, l_blue_;

  DigitalInput *switch_one;
  DigitalInput *switch_two;
  Timer *camera_timer_ = new Timer();

  bool is_wedge_deployed_ = false;
  bool test_flag_;
  bool in_highgear_;
  bool vision_done_ = false;  // UGLY HACK
  bool shootable_ = false;    // ALSO UGLY HACK
  bool start_climb_ = false;  // ANOTHER UGLY HACK
  bool intaking_ = false;
  bool tuck_def_ = false;
  bool disabled_;
  Time time = 0 * s;
  int profiles_run_ = 0;

  CitrusRobot();
  void RobotInit();
  std::string GetAutoRoutine();
  void AutonomousInit();
  void AutonomousPeriodic();
  void TeleopInit();
  void DisabledInit();
  void DisabledPeriodic();
  void TeleopPeriodic();
  void SetDriveGoal(DrivetrainGoal* drivetrain_goal);
  void UpdateLights();
  void ColorLights();
  void SetLightColor(int r, int g, int b);
  void UpdateButtons();
  ~CitrusRobot();
};

#endif
