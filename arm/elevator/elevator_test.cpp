#include "gtest/gtest.h"
#include "elevator_constants.h"
#include "elevator_controller.h"
#include "muan/control/control_utils.h"
#include <fstream>

TEST(ElevatorPlant, isSane) {
  std::cout << ElevatorConstants::A_c << std::endl << ElevatorConstants::B_c << std::endl << ElevatorConstants::C_c << std::endl;
  auto plant = muan::c2d(ElevatorConstants::A_c, ElevatorConstants::B_c, ElevatorConstants::C_c, .01*s);
  for (Time t = 0; t < 1; t += .01 * s) {
    plant.Update(muan::as_matrix<1, 1>({{1}}));
  }
  for (Time t = 0; t < 10; t += .01 * s) {
    plant.Update(muan::as_matrix<1, 1>({{0}}));
  }
  ASSERT_LT(plant.GetX()(0), 1e2);
}

TEST(ElevatorPlant, GoesToZero) {
  auto plant = muan::c2d(ElevatorConstants::A_c, ElevatorConstants::B_c, ElevatorConstants::C_c, .01*s);
  ElevatorController controller(.01*s);
  plant.SetX(muan::as_matrix<2, 1>({{4*ft.to(m)}, {0}}));
  controller.SetGoal(0*m);
  std::ofstream out_file("out_ctl.csv");
  for (Time t = 0; t < 5; t += .01*s) {
    Voltage v = controller.Update(plant.GetY()(0) * m, true);
    plant.Update(muan::as_matrix<1, 1>({{v.to(V)}}));
    out_file << t << ", " << plant.GetX()(0) << ", " << v.to(V) << std::endl;
  }
  out_file.close();
  ASSERT_NEAR(plant.GetX()(0), 0, .01);
}

TEST(ElevatorObserver, TracksInput) {
  auto plant = muan::c2d(ElevatorConstants::A_c, ElevatorConstants::B_c, ElevatorConstants::C_c, .01*s);
  ElevatorController controller(.01*s);
  plant.SetX(muan::as_matrix<2, 1>({{4*ft.to(m)}, {0}}));
  controller.SetGoal(0*m);
  std::ofstream out_file("out_obs.csv");
  for (Time t = 0; t < 5; t += .01*s) {
    Voltage v = controller.Update(plant.GetY()(0) * m, false);
    plant.Update(muan::as_matrix<1, 1>({{0}}));
    out_file << t << ", " << plant.GetX()(0) << ", " << controller.GetObservedState()(0) << std::endl;
  }
  out_file.close();
  ASSERT_NEAR(plant.GetX()(0), controller.GetObservedState()(0), .01);
}

TEST(ElevatorController, GoesToPosition) {
  auto plant = muan::c2d(ElevatorConstants::A_c, ElevatorConstants::B_c, ElevatorConstants::C_c, .01*s);
  ElevatorController controller(.01*s);
  plant.SetX(muan::as_matrix<2,1>({{0*ft.to(m)}, {0}}));
  controller.SetGoal(-1*m);
  std::ofstream out_file("out_file.csv");
  for (Time t = 0; t < 5; t+= .01*s) {
    Voltage v = controller.Update(plant.GetY()(0) * m, true);
    plant.Update(muan::as_matrix<1,1>({{v.to(V)}}));
    out_file << t << ", " << plant.GetX()(0) << ", " << controller.GetObservedState()(0) << ", " << v.to(V) << std::endl;
  }
  out_file.close();
  ASSERT_NEAR(plant.GetX()(0), -1, .01);
}
