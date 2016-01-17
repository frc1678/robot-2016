#include "polydrivetrain_dog_motor_plant.h"

#include <vector>

#include "aos_control/state_feedback_loop.h"

namespace drivetrain {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.963175407007, 0.00630898209772, 0.00630898209772, 0.963175407007;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0141177378239, -0.00241872476926, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.963175407007, 0.00630898209772, 0.00630898209772, 0.963175407007;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0141177378239, -0.00241872476926, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.963175407007, 0.00630898209772, 0.00630898209772, 0.963175407007;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0141177378239, -0.00241872476926, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.963175407007, 0.00630898209772, 0.00630898209772, 0.963175407007;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0141177378239, -0.00241872476926, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.943175407007, 0.00630898209772, 0.00630898209772, 0.943175407007;
  Eigen::Matrix<double, 2, 2> K;
  K << 26.5815523425, 5.00097409617, 5.00097409617, 26.5815523425;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0382770361, -0.00680091205152, -0.00680091205152, 1.0382770361;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.943175407007, 0.00630898209772, 0.00630898209772, 0.943175407007;
  Eigen::Matrix<double, 2, 2> K;
  K << 26.5815523425, 5.00097409617, 5.00097409617, 26.5815523425;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0382770361, -0.00680091205152, -0.00680091205152, 1.0382770361;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.943175407007, 0.00630898209772, 0.00630898209772, 0.943175407007;
  Eigen::Matrix<double, 2, 2> K;
  K << 26.5815523425, 5.00097409617, 5.00097409617, 26.5815523425;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0382770361, -0.00680091205152, -0.00680091205152, 1.0382770361;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.943175407007, 0.00630898209772, 0.00630898209772, 0.943175407007;
  Eigen::Matrix<double, 2, 2> K;
  K << 26.5815523425, 5.00097409617, 5.00097409617, 26.5815523425;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0382770361, -0.00680091205152, -0.00680091205152, 1.0382770361;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<2, 2, 2> MakeVelocityDrivetrainPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>> plants(4);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowLowPlantCoefficients()));
  plants[1] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowHighPlantCoefficients()));
  plants[2] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighLowPlantCoefficients()));
  plants[3] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighHighPlantCoefficients()));
  return StateFeedbackPlant<2, 2, 2>(&plants);
}

StateFeedbackLoop<2, 2, 2> MakeVelocityDrivetrainLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<2, 2, 2>>> controllers(4);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowLowController()));
  controllers[1] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowHighController()));
  controllers[2] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighLowController()));
  controllers[3] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighHighController()));
  return StateFeedbackLoop<2, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace drivetrain
