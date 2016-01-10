#ifndef DRIVETRAIN_CONTROL_LOOPS_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_
#define DRIVETRAIN_CONTROL_LOOPS_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_

#include "aos_control/state_feedback_loop.h"

namespace drivetrain {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2>
MakeVelocityDrivetrainLowLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowLowController();

StateFeedbackPlantCoefficients<2, 2, 2>
MakeVelocityDrivetrainLowHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController();

StateFeedbackPlantCoefficients<2, 2, 2>
MakeVelocityDrivetrainHighLowPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController();

StateFeedbackPlantCoefficients<2, 2, 2>
MakeVelocityDrivetrainHighHighPlantCoefficients();

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController();

StateFeedbackPlant<2, 2, 2> MakeVelocityDrivetrainPlant();

StateFeedbackLoop<2, 2, 2> MakeVelocityDrivetrainLoop();

}  // namespace control_loops
}  // namespace drivetrain

#endif  // DRIVETRAIN_CONTROL_LOOPS_POLYDRIVETRAIN_DOG_MOTOR_PLANT_H_
