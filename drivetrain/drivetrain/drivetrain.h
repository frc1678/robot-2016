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
  virtual void ZeroOutputs();

  // Sets the output to zero.
  // Over-ride if a value of zero is not "off" for this subsystem.
  virtual void Zero(OutputType *output) { output->Zero(); }

  // Runs the loop forever.
  void Run() override;

  // Runs one cycle of the loop.
  void Iterate() override;



 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
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

// All the methods from 971's control loop stuff.
// TODO (jasmine): Strip out all the templating.
template <class T>
void ControlLoop<T>::ZeroOutputs() {
  aos::ScopedMessagePtr<OutputType> output =
      control_loop_->output.MakeMessage();
  Zero(output.get());
  output.Send();
}

template <class T>
void ControlLoop<T>::Iterate() {
  no_goal_.Print();
  driver_station_old_.Print();
  no_sensor_state_.Print();
  no_driver_station_.Print();
  motors_off_log_.Print();

  control_loop_->position.FetchAnother();
  const PositionType *const position = control_loop_->position.get();
  LOG_STRUCT(DEBUG, "position", *position);

  // Fetch the latest control loop goal. If there is no new
  // goal, we will just reuse the old one.
  control_loop_->goal.FetchLatest();
  const GoalType *goal = control_loop_->goal.get();
  if (goal) {
    LOG_STRUCT(DEBUG, "goal", *goal);
  } else {
    LOG_INTERVAL(no_goal_);
  }

  ::aos::robot_state.FetchLatest();
  if (!::aos::robot_state.get()) {
    LOG_INTERVAL(no_sensor_state_);
    return;
  }
  if (sensor_reader_pid_ != ::aos::robot_state->reader_pid) {
    LOG(INFO, "new sensor reader PID %" PRId32 ", old was %" PRId32 "\n",
        ::aos::robot_state->reader_pid, sensor_reader_pid_);
    reset_ = true;
    sensor_reader_pid_ = ::aos::robot_state->reader_pid;
  }

  bool outputs_enabled = false;

  // Check to see if we got a driver station packet recently.
  if (::aos::joystick_state.FetchLatest()) {
    if (::aos::joystick_state->enabled) outputs_enabled = true;
    if (::aos::robot_state->outputs_enabled) {
      // If the driver's station reports being disabled, we're probably not
      // actually going to send motor values regardless of what the FPGA
      // reports.
      if (::aos::joystick_state->enabled) {
        last_pwm_sent_ = ::aos::robot_state->sent_time;
      } else {
        LOG(WARNING, "outputs enabled while disabled\n");
      }
    } else if (::aos::joystick_state->enabled) {
      LOG(WARNING, "outputs disabled while enabled\n");
    }
  } else if (::aos::joystick_state.IsNewerThanMS(kDSPacketTimeoutMs)) {
    if (::aos::joystick_state->enabled) {
      outputs_enabled = true;
    }
  } else {
    if (::aos::joystick_state.get()) {
      LOG_INTERVAL(driver_station_old_);
    } else {
      LOG_INTERVAL(no_driver_station_);
    }
  }

  const bool motors_off =
      (::aos::time::Time::Now() - last_pwm_sent_) >= kPwmDisableTime;
  if (motors_off) {
    if (::aos::joystick_state.get() && ::aos::joystick_state->enabled) {
      LOG_INTERVAL(motors_off_log_);
    }
    outputs_enabled = false;
  }

  aos::ScopedMessagePtr<StatusType> status =
      control_loop_->status.MakeMessage();
  if (status.get() == nullptr) {
    return;
  }

  if (outputs_enabled) {
    aos::ScopedMessagePtr<OutputType> output =
        control_loop_->output.MakeMessage();
    RunIteration(goal, position, output.get(), status.get());

    LOG_STRUCT(DEBUG, "output", *output);
    output.Send();
  } else {
    // The outputs are disabled, so pass nullptr in for the output.
    RunIteration(goal, position, nullptr, status.get());
    ZeroOutputs();
  }

  LOG_STRUCT(DEBUG, "status", *status);
  status.Send();
}

template <class T>
void ControlLoop<T>::Run() {
  while (true) {
    Iterate();
  }
}



}  // namespace control_loops
}  // namespace drivetrain

#endif  // Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
