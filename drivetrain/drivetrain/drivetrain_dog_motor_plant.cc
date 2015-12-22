#include "drivetrain_dog_motor_plant.h"

#include <vector>

#include "drivetrain/state_feedback_loop.h"

namespace drivetrain {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00487799332236, 0.0, -4.83349900286e-05, 0.0, 0.95166140213, 0.0, -0.019015372403, 0.0, -4.83349900286e-05, 1.0, 0.00487799332236, 0.0, -0.019015372403, 0.0, 0.95166140213;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.11831151908e-05, 1.23537136729e-05, 0.0123546357842, 0.0048600499537, 1.23537136729e-05, 3.11831151908e-05, 0.0048600499537, 0.0123546357842;
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
  A << 1.0, 0.00487799332236, 0.0, -4.83349900286e-05, 0.0, 0.95166140213, 0.0, -0.019015372403, 0.0, -4.83349900286e-05, 1.0, 0.00487799332236, 0.0, -0.019015372403, 0.0, 0.95166140213;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.11831151908e-05, 1.23537136729e-05, 0.0123546357842, 0.0048600499537, 1.23537136729e-05, 3.11831151908e-05, 0.0048600499537, 0.0123546357842;
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
  A << 1.0, 0.00487799332236, 0.0, -4.83349900286e-05, 0.0, 0.95166140213, 0.0, -0.019015372403, 0.0, -4.83349900286e-05, 1.0, 0.00487799332236, 0.0, -0.019015372403, 0.0, 0.95166140213;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.11831151908e-05, 1.23537136729e-05, 0.0123546357842, 0.0048600499537, 1.23537136729e-05, 3.11831151908e-05, 0.0048600499537, 0.0123546357842;
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
  A << 1.0, 0.00487799332236, 0.0, -4.83349900286e-05, 0.0, 0.95166140213, 0.0, -0.019015372403, 0.0, -4.83349900286e-05, 1.0, 0.00487799332236, 0.0, -0.019015372403, 0.0, 0.95166140213;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.11831151908e-05, 1.23537136729e-05, 0.0123546357842, 0.0048600499537, 1.23537136729e-05, 3.11831151908e-05, 0.0048600499537, 0.0123546357842;
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
  L << 1.25166140213, -0.019015372403, 73.7324936245, -3.96018826253, -0.019015372403, 1.25166140213, -3.96018826253, 73.7324936245;
  Eigen::Matrix<double, 2, 4> K;
  K << 157.738445792, 12.9116573962, -3.75693684983, -2.06460050147, -3.75693684984, -2.06460050147, 157.738445792, 12.9116573962;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512679767062, 0.0, -5.16496485019e-05, 0.0, 1.05121360084, 0.0, 0.0210045485193, 0.0, -5.16496485019e-05, 1.0, -0.00512679767062, 0.0, 0.0210045485193, 0.0, 1.05121360084;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25166140213, -0.019015372403, 73.7324936245, -3.96018826253, -0.019015372403, 1.25166140213, -3.96018826253, 73.7324936245;
  Eigen::Matrix<double, 2, 4> K;
  K << 157.738445792, 12.9116573962, -3.75693684983, -2.06460050147, -3.75693684984, -2.06460050147, 157.738445792, 12.9116573962;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512679767062, 0.0, -5.16496485019e-05, 0.0, 1.05121360084, 0.0, 0.0210045485193, 0.0, -5.16496485019e-05, 1.0, -0.00512679767062, 0.0, 0.0210045485193, 0.0, 1.05121360084;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25166140213, -0.019015372403, 73.7324936245, -3.96018826253, -0.019015372403, 1.25166140213, -3.96018826253, 73.7324936245;
  Eigen::Matrix<double, 2, 4> K;
  K << 157.738445792, 12.9116573962, -3.75693684983, -2.06460050147, -3.75693684984, -2.06460050147, 157.738445792, 12.9116573962;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512679767062, 0.0, -5.16496485019e-05, 0.0, 1.05121360084, 0.0, 0.0210045485193, 0.0, -5.16496485019e-05, 1.0, -0.00512679767062, 0.0, 0.0210045485193, 0.0, 1.05121360084;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25166140213, -0.019015372403, 73.7324936245, -3.96018826253, -0.019015372403, 1.25166140213, -3.96018826253, 73.7324936245;
  Eigen::Matrix<double, 2, 4> K;
  K << 157.738445792, 12.9116573962, -3.75693684983, -2.06460050147, -3.75693684984, -2.06460050147, 157.738445792, 12.9116573962;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512679767062, 0.0, -5.16496485019e-05, 0.0, 1.05121360084, 0.0, 0.0210045485193, 0.0, -5.16496485019e-05, 1.0, -0.00512679767062, 0.0, 0.0210045485193, 0.0, 1.05121360084;
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
