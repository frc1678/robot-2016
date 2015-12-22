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
  Joystick* j_wheel;
  Joystick* j_stick;

  //Buttonz!
  CitrusButton* shiftDown;
  CitrusButton* shiftUp;
  CitrusButton* quickTurn;


public:
  CitrusRobot() {
    
    //Robot Parts
    drive = new RobotDrive(1, 2);
    drive_loop = new DrivetrainLoop();    
    //shifting = new DoubleSolenoid(1,2);

    //Joysticks
    j_wheel = new Joystick(0); 
    j_stick = new Joystick(1);
    //manipulator = new Joystick(2);

    //Buttonz!
    shiftDown = new CitrusButton(j_stick, 475328);
    shiftUp = new CitrusButton(j_stick, 353);
    quickTurn = new CitrusButton(j_wheel, 321);
    
  }

  void RobotInit() {
    drive->SetSafetyEnabled(false);
  }

  void TeleopInit() {

  }

  void DisabledPeriodic(){
    // TODO (Ash): Stick this in a function so that we don't do all of this multiple times.
    DrivetrainGoal drivetrain_goal;
    DrivetrainPosition drivetrain_position;
    DrivetrainOutput drivetrain_output;
    DrivetrainStatus drivetrain_status;

    SetDriveGoal(&drivetrain_goal);
    SetDrivePosition(&drivetrain_position);

    drive_loop->RunIteration(&drivetrain_goal, &drivetrain_position, &drivetrain_output, &drivetrain_status);
  }

  void TeleopPeriodic() {
    //TODO (Finn): Get this out of the main loop and into its own thread.
    DrivetrainGoal drivetrain_goal;
    DrivetrainPosition drivetrain_position;
    DrivetrainOutput drivetrain_output;
    DrivetrainStatus drivetrain_status;

    SetDriveGoal(&drivetrain_goal);
    SetDrivePosition(&drivetrain_position);

    drive_loop->RunIteration(&drivetrain_goal, &drivetrain_position, &drivetrain_output, &drivetrain_status);
    
    // TODO (Finn): Add these to the DrivetrainGoal (in SetDriveGoal). Set the relevant bools to true on each iteration. Then set the solenoids based on the value of drivetrain_output.
/*    if (shiftUp->ButtonClicked()) {
	shifting->Set(DoubleSolenoid::Value::kReverse);
    } else if (shiftDown->ButtonClicked()) {	
	shifting->Set(DoubleSolenoid::Value::kForward);
    } else {
	shifting->Set(DoubleSolenoid::Value::kOff);
    } 
    if(quickTurn->ButtonPressed()){
    //dunno how to do this...
    }*/

    // TODO (Finn): Also deal with shifting output and with logging from the status.
    drive->TankDrive(drivetrain_output.left_voltage, drivetrain_output.right_voltage);
    UpdateButtons();
  }

  void SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
    drivetrain_goal->steering = j_wheel->GetX(); //TODO (Finn): find the right axis and translate into the right units and direction.
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

  void UpdateButtons(){
  shiftDown->Update();
  shiftUp->Update();
  quickTurn->Update();
  }

  ~CitrusRobot() {

  }
};

START_ROBOT_CLASS(CitrusRobot);
