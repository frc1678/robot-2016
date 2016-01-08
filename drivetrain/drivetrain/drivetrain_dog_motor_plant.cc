#include "drivetrain_dog_motor_plant.h"

#include <vector>

#include "drivetrain/state_feedback_loop.h"

namespace drivetrain {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00479482292314, 0.0, 3.4835409185e-05, 0.0, 0.919101169898, 0.0, 0.0135448598295, 0.0, 3.4835409185e-05, 1.0, 0.00479482292314, 0.0, 0.0135448598295, 0.0, 0.919101169898;
  Eigen::Matrix<double, 4, 2> B;
  B << 5.24402479085e-05, -8.9034190448e-06, 0.0206765530099, -0.00346186727202, -8.9034190448e-06, 5.24402479085e-05, -0.00346186727202, 0.0206765530099;
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
  A << 1.0, 0.00479482292314, 0.0, 3.4835409185e-05, 0.0, 0.919101169898, 0.0, 0.0135448598295, 0.0, 3.4835409185e-05, 1.0, 0.00479482292314, 0.0, 0.0135448598295, 0.0, 0.919101169898;
  Eigen::Matrix<double, 4, 2> B;
  B << 5.24402479085e-05, -8.9034190448e-06, 0.0206765530099, -0.00346186727202, -8.9034190448e-06, 5.24402479085e-05, -0.00346186727202, 0.0206765530099;
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
  A << 1.0, 0.00479482292314, 0.0, 3.4835409185e-05, 0.0, 0.919101169898, 0.0, 0.0135448598295, 0.0, 3.4835409185e-05, 1.0, 0.00479482292314, 0.0, 0.0135448598295, 0.0, 0.919101169898;
  Eigen::Matrix<double, 4, 2> B;
  B << 5.24402479085e-05, -8.9034190448e-06, 0.0206765530099, -0.00346186727202, -8.9034190448e-06, 5.24402479085e-05, -0.00346186727202, 0.0206765530099;
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
  A << 1.0, 0.00479482292314, 0.0, 3.4835409185e-05, 0.0, 0.919101169898, 0.0, 0.0135448598295, 0.0, 3.4835409185e-05, 1.0, 0.00479482292314, 0.0, 0.0135448598295, 0.0, 0.919101169898;
  Eigen::Matrix<double, 4, 2> B;
  B << 5.24402479085e-05, -8.9034190448e-06, 0.0206765530099, -0.00346186727202, -8.9034190448e-06, 5.24402479085e-05, -0.00346186727202, 0.0206765530099;
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
  L << 1.2191011699, 0.0135448598295, 67.0440956124, 2.7282097495, 0.0135448598295, 1.2191011699, 2.7282097495, 67.0440956124;
  Eigen::Matrix<double, 2, 4> K;
  K << 151.453009325, 10.2081129721, 2.52849961714, 0.638943922638, 2.52849961714, 0.638943922638, 151.453009325, 10.2081129721;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00521743543448, 0.0, 3.89881153655e-05, 0.0, 1.08825585389, 0.0, -0.0160377045339, 0.0, 3.89881153655e-05, 1.0, -0.00521743543448, 0.0, -0.0160377045339, 0.0, 1.08825585389;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2191011699, 0.0135448598295, 67.0440956124, 2.7282097495, 0.0135448598295, 1.2191011699, 2.7282097495, 67.0440956124;
  Eigen::Matrix<double, 2, 4> K;
  K << 151.453009325, 10.2081129721, 2.52849961714, 0.638943922638, 2.52849961714, 0.638943922638, 151.453009325, 10.2081129721;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00521743543448, 0.0, 3.89881153655e-05, 0.0, 1.08825585389, 0.0, -0.0160377045339, 0.0, 3.89881153655e-05, 1.0, -0.00521743543448, 0.0, -0.0160377045339, 0.0, 1.08825585389;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2191011699, 0.0135448598295, 67.0440956124, 2.7282097495, 0.0135448598295, 1.2191011699, 2.7282097495, 67.0440956124;
  Eigen::Matrix<double, 2, 4> K;
  K << 151.453009325, 10.2081129721, 2.52849961714, 0.638943922638, 2.52849961714, 0.638943922638, 151.453009325, 10.2081129721;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00521743543448, 0.0, 3.89881153655e-05, 0.0, 1.08825585389, 0.0, -0.0160377045339, 0.0, 3.89881153655e-05, 1.0, -0.00521743543448, 0.0, -0.0160377045339, 0.0, 1.08825585389;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2191011699, 0.0135448598295, 67.0440956124, 2.7282097495, 0.0135448598295, 1.2191011699, 2.7282097495, 67.0440956124;
  Eigen::Matrix<double, 2, 4> K;
  K << 151.453009325, 10.2081129721, 2.52849961714, 0.638943922638, 2.52849961714, 0.638943922638, 151.453009325, 10.2081129721;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00521743543448, 0.0, 3.89881153655e-05, 0.0, 1.08825585389, 0.0, -0.0160377045339, 0.0, 3.89881153655e-05, 1.0, -0.00521743543448, 0.0, -0.0160377045339, 0.0, 1.08825585389;
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
