#include <WPILib.h>

#include "drivetrain/drivetrain/drivetrain.h"

using drivetrain::control_loops::DrivetrainGoal;
using drivetrain::control_loops::DrivetrainPosition;
using drivetrain::control_loops::DrivetrainOutput;
using drivetrain::control_loops::DrivetrainStatus;

class CitrusRobot : public IterativeRobot {
  RobotDrive* drive;
  DrivetrainLoop* drive_loop;
  Joystick* j_wheel;
  Joystick* j_stick;
public:
  CitrusRobot() {
    drive = new RobotDrive(1, 2);
    drive_loop = new DrivetrainLoop();
    j_wheel = new Joystick(0); // TODO (Finn): Get the axis from the steering wheel and get it into the appropriate units.
    j_stick = new Joystick(1);
  }
  void RobotInit() {

  }
  void TeleopInit() {

  }
  void TeleopPeriodic() {
    //TODO (Finn): Get this out of the main loop and into its own thread.
    DrivetrainGoal drivetrain_goal;
    DrivetrainPosition drivetrain_position;
    DrivetrainOutput drivetrain_output;
    DrivetrainStatus drivetrain_status;

    SetDriveGoal(&drivetrain_goal);
    SetDrivePosition(&drivetrain_position);

    // TODO (Finn): Also run the iteration, but with no output, when we're disabled.
    drive_loop->RunIteration(&drivetrain_goal, &drivetrain_position, &drivetrain_output, &drivetrain_status);

    // TODO (Finn): Also deal with shifting output and with logging from the status.
    drive->TankDrive(drivetrain_output->left_voltage, drivetrain_output->right_voltage);
  }

  void SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
    drivetrain_goal->steering = j_wheel->GetY(); //TODO (Finn): find the right axis and translate into the right units and direction.
    // The right units should be -1 to 1 from right to left I think? TODO (jasmine): ask Austin about units
    drivetrain_goal->steering = j_stick->GetY();
    drivetrain_goal->highgear = false; // TODO (Finn): Throw this on a button (toggle or two buttons to switch).
    drivetrain_goal->quickturn = false; // TODO (Finn): Throw this on a button (press to true)
    drivetrain_goal->control_loop_driving = false;
  }
  void SetDrivePosition(DrivetrainPosition *drivetrain_position) {
    drivetrain_position->left_encoder = 0; // TODO (Finn): Get this from the encoders in the right units and direction.
    drivetrain_position->right_encoder = 0;

    drivetrain_position->left_shifter_high = false;
    drivetrain_position->right_shifter_high = false;
  }

  ~CitrusRobot() {

  }
};

START_ROBOT_CLASS(CitrusRobot);
