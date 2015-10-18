#ifndef CONTROL_LOOPS_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_DOG_MOTOR_PLANT_H_
#define CONTROL_LOOPS_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_DOG_MOTOR_PLANT_H_

#include "Drivecode/control_loops/state_feedback_loop.h"

namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeDrivetrainLowLowController();

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowHighPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController();

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighLowPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController();

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighHighPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController();

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant();

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop();

}  // namespace control_loops

#endif  // CONTROL_LOOPS_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_DOG_MOTOR_PLANT_H_
