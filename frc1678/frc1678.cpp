#include <WPILib.h>
#include <memory>

#include "drivetrain/drivetrain/drivetrain.h"
#include "CitrusButton.h"
#include "gyro_reader.h"

using drivetrain::control_loops::DrivetrainGoal;
using drivetrain::control_loops::DrivetrainPosition;
using drivetrain::control_loops::DrivetrainOutput;
using drivetrain::control_loops::DrivetrainStatus;
using drivetrain::control_loops::DrivetrainLoop;

class CitrusRobot : public IterativeRobot {
        std::unique_ptr<RobotDrive> drive_;
        std::unique_ptr<DrivetrainLoop> drive_loop_;
        std::unique_ptr<Encoder> left_encoder_, right_encoder_;
        std::unique_ptr<Joystick> j_wheel_, j_stick_;
        std::unique_ptr<DoubleSolenoid> shifting_;
        std::unique_ptr<GyroReader> gyro_reader_;

        // Buttonz!
        std::unique_ptr<CitrusButton> shift_down_, shift_up_, quick_turn_;

        // Pirmitive types
        bool in_highgear_;

       public:
        CitrusRobot() {
                // Robot Parts
                drive_ = std::make_unique<RobotDrive>(2, 1);
                drive_loop_ = std::make_unique<DrivetrainLoop>();
                left_encoder_ = std::make_unique<Encoder>(12, 13);
                right_encoder_ = std::make_unique<Encoder>(10, 11);

                gyro_reader_ = std::make_unique<GyroReader>();

                // Joysticks
                j_wheel_ = std::make_unique<Joystick>(0);
                j_stick_ = std::make_unique<Joystick>(1);
                // manipulator = new Joystick(2);

                shifting_ = std::make_unique<DoubleSolenoid>(1, 2);

                // Buttonz!
                shift_down_ = std::make_unique<CitrusButton>(j_stick_.get(), 2);
                shift_up_ = std::make_unique<CitrusButton>(j_stick_.get(), 1);
                quick_turn_ = std::make_unique<CitrusButton>(j_wheel_.get(), 5);

                in_highgear_ = false;
        }

        void RobotInit() {
                drive_->SetSafetyEnabled(false);
                gyro_reader_->Start();
        }

        void TeleopInit() {}

        void DisabledPeriodic() {
                // TODO (Finn): Get this out of the main loop and into its own
                // thread.
                DrivetrainGoal drivetrain_goal;
                DrivetrainPosition drivetrain_position;
                DrivetrainOutput drivetrain_output;
                DrivetrainStatus drivetrain_status;

                SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
                SmartDashboard::PutNumber("Stick", j_stick_->GetY());
                SetDriveGoal(&drivetrain_goal);
                SetDrivePosition(&drivetrain_position);

                drive_loop_->RunIteration(
                    &drivetrain_goal, &drivetrain_position, &drivetrain_output,
                    &drivetrain_status);
                SmartDashboard::PutNumber("Left voltage",
                                          drivetrain_output.left_voltage);
                SmartDashboard::PutNumber("Right voltage",
                                          drivetrain_output.right_voltage);
        }

        void TeleopPeriodic() {
                // TODO (Finn): Get this out of the main loop and into its own
                // thread.
                DrivetrainGoal drivetrain_goal;
                DrivetrainPosition drivetrain_position;
                DrivetrainOutput drivetrain_output;
                DrivetrainStatus drivetrain_status;

                SmartDashboard::PutNumber("Wheel", j_wheel_->GetX());
                SmartDashboard::PutNumber("Stick", j_stick_->GetY());

                // TODO (Finn): Act on the output, without bypassing the
                // controller. Or argue that this is fine.
                if (shift_up_->ButtonClicked()) {
                        shifting_->Set(DoubleSolenoid::Value::kReverse);
                        in_highgear_ = true;
                } else if (shift_down_->ButtonClicked()) {
                        shifting_->Set(DoubleSolenoid::Value::kForward);
                        in_highgear_ = false;
                } else {
                        shifting_->Set(DoubleSolenoid::Value::kOff);
                }

                SetDriveGoal(&drivetrain_goal);
                SetDrivePosition(&drivetrain_position);

                drive_loop_->RunIteration(
                    &drivetrain_goal, &drivetrain_position, &drivetrain_output,
                    &drivetrain_status);
                SmartDashboard::PutNumber("Left voltage",
                                          drivetrain_output.left_voltage);
                SmartDashboard::PutNumber("Right voltage",
                                          drivetrain_output.right_voltage);
                SmartDashboard::PutNumber("Left high",
                                          drivetrain_output.left_high);

                // TODO (Finn): Also deal with shifting output and with logging
                // from the status.
                drive_->TankDrive(drivetrain_output.left_voltage / 12.0,
                                  drivetrain_output.right_voltage / 12.0);
                UpdateButtons();
        }

        void SetDriveGoal(DrivetrainGoal* drivetrain_goal) {
                drivetrain_goal->steering = j_wheel_->GetX();
                drivetrain_goal->throttle = j_stick_->GetY();
                drivetrain_goal->highgear = in_highgear_;
                drivetrain_goal->quickturn = quick_turn_->ButtonPressed();
                drivetrain_goal->control_loop_driving = false;
        }

        void SetDrivePosition(DrivetrainPosition* drivetrain_position) {
                double click =
                    3.14159 * .1016 /
                    360.0;  // Translating encoders into ground distances.
                drivetrain_position->left_encoder =
                    left_encoder_->Get() * click;  // TODO (Ash): Get this from
                                                   // the encoders in the right
                                                   // units and direction.
                drivetrain_position->right_encoder =
                    -right_encoder_->Get() * click;
                drivetrain_position->gyro_angle =
                    gyro_reader_->GetAngle().to(rad);
                drivetrain_position->left_shifter_high = in_highgear_;
                drivetrain_position->right_shifter_high = in_highgear_;
        }

        void UpdateButtons() {
                shift_down_->Update();
                shift_up_->Update();
                quick_turn_->Update();
        }

        ~CitrusRobot() {}
};

START_ROBOT_CLASS(CitrusRobot);
