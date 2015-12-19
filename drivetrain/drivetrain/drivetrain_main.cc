#include "y2015_bot3/control_loops/drivetrain/drivetrain.h"

#include "aos/linux_code/init.h"

int main() {
  ::muan::Init();
  ::y2015_bot3::control_loops::DrivetrainLoop drivetrain;//TODO 
  drivetrain.Run();
  ::muan::Cleanup();
  return 0;
}
