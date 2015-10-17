// With apologies for stripping out all the useful stuff.

class ControlLoop {
 public:
  ControlLoop() {}
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

// What we'll do is just "send everything to a safe state."
// TODO (Jasmine): actually implement this for 1678.
  virtual void ZeroOutputs();
  // Sets the output to zero.
  // Over-ride if a value of zero is not "off" for this subsystem.
//  virtual void Zero(OutputType *output) { output->Zero(); }

  // Runs the loop forever.
  void Run(){
while(true) { Iterate();}
}

  // Runs one cycle of the loop.
  void Iterate();

  protected:
  // Runs an iteration of the control loop.
  // goal is the last goal that was sent.  It might be any number of cycles old
  // or nullptr if we haven't ever received a goal.
  // position is the current position, or nullptr if we didn't get a position
  // this cycle.
  // output is the values to be sent to the motors.  This is nullptr if the
  // output is going to be ignored and set to 0.
  // status is the status of the control loop.
  // Both output and status should be filled in by the implementation.
  virtual void RunIteration(const GoalType *goal,
                            const PositionType *position,
                            OutputType *output,
                            StatusType *status) = 0;

// Are private variables inherited? Which ones are used?

  bool reset_ = false;
};

//TODO (Jasmine): Strip things out.
// Basically the order goes:
// get a new goal (From joysticks) if there is one, otherwise use the old one.
// get all the sensor inputs
// Check to make sure that the robot is enabled
//     Run an iteration of the control loop.
//     If the robot is enabled, do stuff to the motors.
//     Otherwise, send 0 for the outputs (but you still want to have iterated the control loop.
void Iterate() {
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
