#ifndef Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
#define Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_

#include <string.h>
#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"

#include "y2015_bot3/control_loops/drivetrain/drivetrain.q.h"

namespace drivetrain {
namespace control_loops {

// Constants
// TODO(jasmine): Figure out what these refer to and get actual constants.
constexpr double kDrivetrainTurnWidth = 0.63;
constexpr double kDrivetrainDoneDistance = 0.02;
constexpr double kDrivetrainEncoderRatio = 18.0 / 44.0;
constexpr double kDrivetrainHighGearRatio =
    kDrivetrainEncoderRatio * 18.0 / 60.0;
constexpr double kDrivetrainLowGearRatio = kDrivetrainHighGearRatio;
const bool kDrivetrainClutchTransmission = false;
// End constants

class DrivetrainLoop {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at drivetrain::control_loops::drivetrain
/*  explicit DrivetrainLoop(control_loops::DrivetrainQueue *my_drivetrain =
                              &control_loops::drivetrain_queue)
      : aos::controls::ControlLoop<control_loops::DrivetrainQueue>(
            my_drivetrain) {
    ::aos::controls::HPolytope<0>::Init();
  }*/

  // Returns true if all the counters etc in the sensor data have been reset.
  // This will return true only a single time per reset.
  bool WasReset() {
    if (reset_) {
      reset_ = false;
      return true;
    } else {
      return false;
    }
  }

  // Constructs and sends a message on the output queue which sets everything to
  // a safe state (generally motors off). For some subclasses, this will be a
  // bit different (ie pistons).
  // The implementation here creates a new Output message, calls Zero() on it,
  // and then sends it.
  // TODO (jasmine): Send everything to a safe state, usually 'motors off', without the 971 idiom.
  void ZeroOutputs();

  // Runs the loop forever.
  void Run() override;

  // Runs one cycle of the loop.
  void Iterate() override;

  // Executes one cycle of the control loop.
  void RunIteration(
      const control_loops::DrivetrainQueue::Goal *goal,
      const control_loops::DrivetrainQueue::Position *position,
      control_loops::DrivetrainQueue::Output *output,
      control_loops::DrivetrainQueue::Status *status);

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_position_ = SimpleLogInterval(
      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");

 private:
  bool reset_ = false;
};

// TODO (Finn): move these to the .cc file
// TODO (Finn): Rewrite to do what we want it to do; eg send to a safe state. This depends on how we're implementing the DrivetrainLop.
void DrivetrainLoop::ZeroOutputs() {
}

// TODO (jasmine): Rewrite iterate to do what we want it to do.
// 1. Get what the current position of the robot is (and log it)
// 2. Get the latest goal (and log it) from joysticks?
// 3. Get the current sensor data.
// 4. If we're enabled:
//        run an iteration of the loop and then send the new info to the robot.
//    If we're not:
//        run an iteration of the loop and then tell the robot to enter a safe state.
void DrivetrainLoop::Iterate() {
}

// TODO (Finn): How are we implementing this? Does this need to be called at all?
void DrivetrainLoop::Run() {
  while (true) {
    Iterate();
  }
}



}  // namespace control_loops
}  // namespace drivetrain

#endif  // Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
