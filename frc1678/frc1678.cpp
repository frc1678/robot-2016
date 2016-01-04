#include <WPILib.h>

#include "drivetrain/drivetrain/drivetrain.h"
#include "CitrusButton.h"

using drivetrain::control_loops::DrivetrainGoal;
using drivetrain::control_loops::DrivetrainPosition;
using drivetrain::control_loops::DrivetrainOutput;
using drivetrain::control_loops::DrivetrainStatus;
using drivetrain::control_loops::DrivetrainLoop;

class CitrusRobot : public IterativeRobot {
  RobotDrive* drive;
  DrivetrainLoop* drive_loop;
  Encoder *left_encoder;
  Encoder *right_encoder;
  Joystick* j_wheel;
  Joystick* j_stick;
  DoubleSolenoid *shifting;

  //Buttonz!
  CitrusButton* shiftDown;
  CitrusButton* shiftUp;
  CitrusButton* quickTurn;

  // Pirmitive types
  bool in_highgear;

public:
  CitrusRobot() {

    //Robot Parts
    drive = new RobotDrive(2, 1);
    drive_loop = new DrivetrainLoop();
    //shifting = new DoubleSolenoid(1,2);
    left_encoder = new Encoder(12, 13);
    right_encoder = new Encoder(10, 11);


    //Joysticks
    j_wheel = new Joystick(0);
    j_stick = new Joystick(1);
    //manipulator = new Joystick(2);

    shifting = new DoubleSolenoid(1, 2);

    //Buttonz!
    shiftDown = new CitrusButton(j_stick, 2);
    shiftUp = new CitrusButton(j_stick, 1);
    quickTurn = new CitrusButton(j_wheel, 5);

    in_highgear = false;
  }

  void RobotInit() {
    drive->SetSafetyEnabled(false);
  }

  void TeleopInit() {

  }

  void DisabledPeriodic(){
    //TODO (Finn): Get this out of the main loop and into its own thread.
    DrivetrainGoal drivetrain_goal;
    DrivetrainPosition drivetrain_position;
    DrivetrainOutput drivetrain_output;
    DrivetrainStatus drivetrain_status;

    SmartDashboard::PutNumber("Wheel", j_wheel->GetX());
    SmartDashboard::PutNumber("Stick", j_stick->GetY());
    SetDriveGoal(&drivetrain_goal);
    SetDrivePosition(&drivetrain_position);

    drive_loop->RunIteration(&drivetrain_goal, &drivetrain_position, &drivetrain_output, &drivetrain_status);
    SmartDashboard::PutNumber("Left voltage", drivetrain_output.left_voltage);
    SmartDashboard::PutNumber("Right voltage", drivetrain_output.right_voltage);
  }

  void TeleopPeriodic() {

    //TODO (Finn): Get this out of the main loop and into its own thread.
    DrivetrainGoal drivetrain_goal;
    DrivetrainPosition drivetrain_position;
    DrivetrainOutput drivetrain_output;
    DrivetrainStatus drivetrain_status;

    SmartDashboard::PutNumber("Wheel", j_wheel->GetX());
    SmartDashboard::PutNumber("Stick", j_stick->GetY());

    // TODO (Finn): Act on the output, without bypassing the controller. Or argue that this is fine.
    if (shiftUp->ButtonClicked()) {
	shifting->Set(DoubleSolenoid::Value::kReverse);
        in_highgear = true;
    } else if (shiftDown->ButtonClicked()) {
	shifting->Set(DoubleSolenoid::Value::kForward);
        in_highgear = false;
    } else {
	shifting->Set(DoubleSolenoid::Value::kOff);
    }


    SetDriveGoal(&drivetrain_goal);
    SetDrivePosition(&drivetrain_position);

    drive_loop->RunIteration(&drivetrain_goal, &drivetrain_position, &drivetrain_output, &drivetrain_status);
    SmartDashboard::PutNumber("Left voltage", drivetrain_output.left_voltage);
    SmartDashboard::PutNumber("Right voltage", drivetrain_output.right_voltage);
    SmartDashboard::PutNumber("Left high", drivetrain_output.left_high);

    // TODO (Finn): Also deal with shifting output and with logging from the status.
    drive->TankDrive(drivetrain_output.left_voltage/12.0, drivetrain_output.right_voltage/12.0);
    UpdateButtons();
  }

  void SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
    drivetrain_goal->steering = j_wheel->GetX();
    drivetrain_goal->throttle = j_stick->GetY();
    drivetrain_goal->highgear = in_highgear;
    drivetrain_goal->quickturn = quickTurn->ButtonPressed();
    drivetrain_goal->control_loop_driving = false;
  }

  void SetDrivePosition(DrivetrainPosition *drivetrain_position) {
    double click = 3.14159 * .1016 / 360.0; // Translating encoders into ground distances.
    drivetrain_position->left_encoder = left_encoder->Get() * click; // TODO (Ash): Get this from the encoders in the right units and direction.
    drivetrain_position->right_encoder = -right_encoder->Get() * click;
    drivetrain_position->left_shifter_high = in_highgear;
    drivetrain_position->right_shifter_high = in_highgear;
  }

  void UpdateButtons(){
  shiftDown->Update();
  shiftUp->Update();
  quickTurn->Update();
  }

  ~CitrusRobot() {
    // TODO (Ash): Finish writing the destructor.
    delete drive;
    delete drive_loop;
    delete left_encoder;
    delete right_encoder;
  }

};

START_ROBOT_CLASS(CitrusRobot);
