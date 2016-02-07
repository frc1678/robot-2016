#ifndef FRC1678_H
#define FRC1678_H

#include "CitrusButton.h"
#include "frc1678/auto/auto_routines.h"
#include "vision/vision.h"
#include "robot_subsystems.h"

//class LemonScriptRunner { };

class CitrusRobot : public IterativeRobot {
 private:
  LemonScriptRunner* auto_runner;

 public:
  std::unique_ptr<Joystick> j_wheel_, j_stick_;

  RobotSubsystems subsystems_;
  CitrusVision vision_;

  // Buttonz!
  std::unique_ptr<CitrusButton> shift_down_, shift_up_, quick_turn_;
  
  bool test_flag_;
  bool in_highgear_;
  bool vision_done_ = false; //UGLY HACK

  CitrusRobot();
  void RobotInit();
  void AutonomousInit();
  void AutonomousPeriodic();
  void TeleopInit();
  void DisabledPeriodic();
  void TeleopPeriodic();
  void SetDriveGoal(DrivetrainGoal* drivetrain_goal);
  void UpdateButtons();
  ~CitrusRobot();
};


#endif
