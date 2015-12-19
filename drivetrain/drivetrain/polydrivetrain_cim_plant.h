#ifndef DRIVETRAIN_CONTROL_LOOPS_POLYDRIVETRAIN_CIM_PLANT_H_
#define DRIVETRAIN_CONTROL_LOOPS_POLYDRIVETRAIN_CIM_PLANT_H_

#include "drivetrain/state_feedback_loop.h"

namespace drivetrain {
namespace control_loops {

StateFeedbackPlantCoefficients<1, 1, 1> MakeCIMPlantCoefficients();

StateFeedbackController<1, 1, 1> MakeCIMController();

StateFeedbackPlant<1, 1, 1> MakeCIMPlant();

StateFeedbackLoop<1, 1, 1> MakeCIMLoop();

}  // namespace control_loops
}  // namespace drivetrain

#endif  // DRIVETRAIN_CONTROL_LOOPS_POLYDRIVETRAIN_CIM_PLANT_H_
