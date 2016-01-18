#include "drivetrain_dog_motor_plant.h"

#include <vector>

#include "aos_control/state_feedback_loop.h"

namespace drivetrain {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00490734529823, 0.0, 1.59716364921e-05, 0.0, 0.963175407007, 0.0, 0.00630898209772, 0.0, 1.59716364921e-05, 1.0, 0.00490734529823, 0.0, 0.00630898209772, 0.0, 0.963175407007;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.55217717693e-05, -6.12317362624e-06, 0.0141177378239, -0.00241872476926, -6.12317362624e-06, 3.55217717693e-05, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00490734529823, 0.0, 1.59716364921e-05, 0.0, 0.963175407007, 0.0, 0.00630898209772, 0.0, 1.59716364921e-05, 1.0, 0.00490734529823, 0.0, 0.00630898209772, 0.0, 0.963175407007;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.55217717693e-05, -6.12317362624e-06, 0.0141177378239, -0.00241872476926, -6.12317362624e-06, 3.55217717693e-05, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00490734529823, 0.0, 1.59716364921e-05, 0.0, 0.963175407007, 0.0, 0.00630898209772, 0.0, 1.59716364921e-05, 1.0, 0.00490734529823, 0.0, 0.00630898209772, 0.0, 0.963175407007;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.55217717693e-05, -6.12317362624e-06, 0.0141177378239, -0.00241872476926, -6.12317362624e-06, 3.55217717693e-05, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00490734529823, 0.0, 1.59716364921e-05, 0.0, 0.963175407007, 0.0, 0.00630898209772, 0.0, 1.59716364921e-05, 1.0, 0.00490734529823, 0.0, 0.00630898209772, 0.0, 0.963175407007;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.55217717693e-05, -6.12317362624e-06, 0.0141177378239, -0.00241872476926, -6.12317362624e-06, 3.55217717693e-05, -0.00241872476926, 0.0141177378239;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.26317540701, 0.00630898209772, 76.1109389055, 1.32890772618, 0.00630898209772, 1.26317540701, 1.32890772618, 76.1109389055;
  Eigen::Matrix<double, 2, 4> K;
  K << 155.247108308, 12.7719045573, 1.97964609729, 0.780764905983, 1.97964609728, 0.780764905983, 155.247108308, 12.7719045573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00509507530967, 0.0, 1.67914403809e-05, 0.0, 1.0382770361, 0.0, -0.00680091205152, 0.0, 1.67914403809e-05, 1.0, -0.00509507530967, 0.0, -0.00680091205152, 0.0, 1.0382770361;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.26317540701, 0.00630898209772, 76.1109389055, 1.32890772618, 0.00630898209772, 1.26317540701, 1.32890772618, 76.1109389055;
  Eigen::Matrix<double, 2, 4> K;
  K << 155.247108308, 12.7719045573, 1.97964609729, 0.780764905983, 1.97964609728, 0.780764905983, 155.247108308, 12.7719045573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00509507530967, 0.0, 1.67914403809e-05, 0.0, 1.0382770361, 0.0, -0.00680091205152, 0.0, 1.67914403809e-05, 1.0, -0.00509507530967, 0.0, -0.00680091205152, 0.0, 1.0382770361;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.26317540701, 0.00630898209772, 76.1109389055, 1.32890772618, 0.00630898209772, 1.26317540701, 1.32890772618, 76.1109389055;
  Eigen::Matrix<double, 2, 4> K;
  K << 155.247108308, 12.7719045573, 1.97964609729, 0.780764905983, 1.97964609728, 0.780764905983, 155.247108308, 12.7719045573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00509507530967, 0.0, 1.67914403809e-05, 0.0, 1.0382770361, 0.0, -0.00680091205152, 0.0, 1.67914403809e-05, 1.0, -0.00509507530967, 0.0, -0.00680091205152, 0.0, 1.0382770361;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.26317540701, 0.00630898209772, 76.1109389055, 1.32890772618, 0.00630898209772, 1.26317540701, 1.32890772618, 76.1109389055;
  Eigen::Matrix<double, 2, 4> K;
  K << 155.247108308, 12.7719045573, 1.97964609729, 0.780764905983, 1.97964609728, 0.780764905983, 155.247108308, 12.7719045573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00509507530967, 0.0, 1.67914403809e-05, 0.0, 1.0382770361, 0.0, -0.00680091205152, 0.0, 1.67914403809e-05, 1.0, -0.00509507530967, 0.0, -0.00680091205152, 0.0, 1.0382770361;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>> plants(4);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowLowPlantCoefficients()));
  plants[1] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowHighPlantCoefficients()));
  plants[2] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighLowPlantCoefficients()));
  plants[3] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighHighPlantCoefficients()));
  return StateFeedbackPlant<4, 2, 2>(&plants);
}

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<4, 2, 2>>> controllers(4);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowLowController()));
  controllers[1] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowHighController()));
  controllers[2] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighLowController()));
  controllers[3] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighHighController()));
  return StateFeedbackLoop<4, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace drivetrain
