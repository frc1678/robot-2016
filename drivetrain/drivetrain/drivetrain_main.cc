/* 
Many thanks to 971 Spartan Robotics for their help with this project
*/
#include "y2015_bot3/control_loops/drivetrain/drivetrain.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2015_bot3::control_loops::DrivetrainLoop drivetrain;//TODO 
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
