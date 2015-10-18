#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "Drivecode/polytope.h"
//#include "aos/common/util/log_interval.h"

#include "Drivecode/shifter_hall_effect.h"

namespace control_loops {

// Constants
// TODO(comran): Get actual constants.
constexpr double kDrivetrainTurnWidth = 0.63;
constexpr double kDrivetrainDoneDistance = 0.02;
constexpr double kDrivetrainEncoderRatio = 18.0 / 44.0;
constexpr double kDrivetrainHighGearRatio =
    kDrivetrainEncoderRatio * 18.0 / 60.0;
constexpr double kDrivetrainLowGearRatio = kDrivetrainHighGearRatio;
const bool kDrivetrainClutchTransmission = false;
const ::frc971::constants::ShifterHallEffect kDrivetrainRightShifter{
    555, 657, 660, 560, 0.2, 0.7};
const ::frc971::constants::ShifterHallEffect kDrivetrainLeftShifter{
    555, 660, 644, 552, 0.2, 0.7};
// End constants

struct DrivetrainGoal {
	double steering;
	double throttle;
	bool highgear;
	bool quickturn;
	bool control_loop_driving;
	double left_goal;
	double left_velocity_goal;
	double right_goal;
	double right_velocity_goal;
};

struct DrivetrainPosition {
	double left_encoder;
	double right_encoder;
	double left_shifter_position;
	double right_shifter_position;
};

struct DrivetrainOutput {
	double left_voltage;
	double right_voltage;
	bool left_high;
	bool right_high;
};

struct DrivetrainStatus {
	double robot_speed;
	double filtered_left_position;
	double filtered_right_position;
	double filtered_left_velocity;
	double filtered_right_velocity;

	double uncapped_left_voltage;
	double uncapped_right_voltage;
	bool output_was_capped;

	bool is_done;
};

class DrivetrainLoop {
public:
	// Constructs a control loop which can take a Drivetrain or defaults to the
	// drivetrain at bot3::control_loops::drivetrain
	explicit DrivetrainLoop() {
		::aos::controls::HPolytope<0>::Init();
	}

	virtual ~DrivetrainLoop();
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
	void Run() {
		while (true) {
			Iterate();
		}
	}

	// Runs one cycle of the loop.
	void Iterate();
protected:
	// Executes one cycle of the control loop.
	virtual void RunIteration(const DrivetrainGoal *goal,
			const DrivetrainPosition *position, DrivetrainOutput *output,
			DrivetrainStatus *status);

private:

	bool reset_ = false;

// From 971's logging.
//  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
//  SimpleLogInterval no_position_ = SimpleLogInterval(
//      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");
};

//TODO (Jasmine): Rewrite Iterate to take care of everything.
// Basically the order goes:
// get a new goal (From joysticks) if there is one, otherwise use the old one.
// get all the sensor inputs
// Check to make sure that the robot is enabled
//     Run an iteration of the control loop.
//     If the robot is enabled, do stuff to the motors.
//     Otherwise, send 0 for the outputs (but you still want to have iterated the control loop.
void DrivetrainLoop::Iterate() {

}

}  // namespace control_loops

#endif  // BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
