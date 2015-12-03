#include <WPILib.h>

class CitrusRobot : public IterativeRobot {
  RobotDrive* drive;
  Joystick* j1;
  Joystick* j2;
public:
  CitrusRobot() {
    drive = new RobotDrive(1, 2);
    j1 = new Joystick(0);
    j2 = new Joystick(1);
  }
  void RobotInit() {

  }
  void TeleopInit() {

  }
  void TeleopPeriodic() {
    drive->TankDrive(j1->GetY(), j2->GetY());
  }
  ~CitrusRobot() {

  }
};

START_ROBOT_CLASS(CitrusRobot);
