/*
Many thanks to 971 Spartan Robotics for their help with this project
*/
#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
//#include "aos/common/network/team_number.h"
//#include "aos/common/queue_testutils.h"
//#include "aos/common/controls/polytope.h"
//#include "aos/common/controls/control_loop_test.h"

#include "drivetrain.h"
#include "aos_control/state_feedback_loop.h"
#include "aos_control/coerce_goal.h"
#include "drivetrain_dog_motor_plant.h"

using drivetrain::control_loops::DrivetrainGoal;
using drivetrain::control_loops::DrivetrainPosition;
using drivetrain::control_loops::DrivetrainOutput;
using drivetrain::control_loops::DrivetrainStatus;
using drivetrain::control_loops::DrivetrainLoop;
using drivetrain::control_loops::MakeDrivetrainPlant;

namespace y2015_bot3 {
namespace control_loops {
namespace testing {

class Environment : public ::testing::Environment {
 public:
  virtual ~Environment() {}
  // how to set up the environment.
  virtual void SetUp() { aos::controls::HPolytope<0>::Init(); }
};

// Class which simulates the drivetrain
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation()
      : drivetrain_plant_(new StateFeedbackPlant<4, 2, 2>(
            MakeDrivetrainPlant())) {  // TODO (Finn): zero-initialize the
                                       // structs
    Reinitialize();
  }

  // Resets the plant.
  void Reinitialize() {
    drivetrain_plant_->mutable_X(0, 0) = 0.0;
    drivetrain_plant_->mutable_X(1, 0) = 0.0;
    drivetrain_plant_->mutable_Y() =
        drivetrain_plant_->C() * drivetrain_plant_->X();
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
  }

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_->Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_->Y(1, 0); }

  // Sends out the position queue messages.
  void SendPositionMessage(DrivetrainPosition* drivetrain_position) {
    const double left_encoder = GetLeftPosition();
    const double right_encoder = GetRightPosition();

    drivetrain_position->left_encoder = left_encoder;
    drivetrain_position->right_encoder = right_encoder;
  }

  // Simulates the drivetrain moving for one timestep.
  // TODO (jasmine): Stop trusting that the output actually changes.
  void Simulate(const DrivetrainOutput* drivetrain_output) {
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
    drivetrain_plant_->mutable_U() << drivetrain_output->left_voltage,
        drivetrain_output->right_voltage;
    drivetrain_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> drivetrain_plant_;

 private:
  double last_left_position_;
  double last_right_position_;
};

class DrivetrainTest : public ::testing::Test {
 protected:
  DrivetrainGoal drivetrain_goal_;
  DrivetrainPosition drivetrain_position_;
  DrivetrainOutput drivetrain_output_;
  DrivetrainStatus drivetrain_status_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  DrivetrainTest()
      :  // TODO (Finn): initialize your structs
        drivetrain_motor_(),
        drivetrain_motor_plant_() {}

  void VerifyNearGoal() {
    EXPECT_NEAR(drivetrain_goal_.left_goal,
                drivetrain_motor_plant_.GetLeftPosition(), 1e-2);
    EXPECT_NEAR(drivetrain_goal_.right_goal,
                drivetrain_motor_plant_.GetRightPosition(), 1e-2);
  }

  virtual ~DrivetrainTest() {}
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  // Zero before setting it up.
  drivetrain_goal_.control_loop_driving = true;
  drivetrain_goal_.left_goal = (-1.0);
  drivetrain_goal_.right_goal = (1.0);
  for (int i = 0; i < 200; ++i) {
    drivetrain_motor_plant_.SendPositionMessage(&drivetrain_position_);
    drivetrain_motor_.RunIteration(&drivetrain_goal_, &drivetrain_position_,
                                   &drivetrain_output_, &drivetrain_status_);
    drivetrain_motor_plant_.Simulate(&drivetrain_output_);
    //    SimulateTimestep(true);
  }
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  drivetrain_goal_.control_loop_driving = true;
  drivetrain_goal_.left_goal = (-1.0);
  drivetrain_goal_.right_goal = (1.0);
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage(&drivetrain_position_);
    drivetrain_motor_.RunIteration(&drivetrain_goal_, &drivetrain_position_,
                                   &drivetrain_output_, &drivetrain_status_);
    drivetrain_motor_plant_.Simulate(&drivetrain_output_);
    if (i > 20 && i < 200) {
      //      SimulateTimestep(false);
    } else {
      //     SimulateTimestep(true);
    }
  }
  VerifyNearGoal();
}

// Tests that never having a goal doesn't break.
TEST_F(DrivetrainTest, NoGoalStart) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage(&drivetrain_position_);
    drivetrain_motor_.RunIteration(&drivetrain_goal_, &drivetrain_position_,
                                   &drivetrain_output_, &drivetrain_status_);
    drivetrain_motor_plant_.Simulate(&drivetrain_output_);
  }
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(DrivetrainTest, NoGoalWithRobotState) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage(&drivetrain_position_);
    drivetrain_motor_.RunIteration(&drivetrain_goal_, &drivetrain_position_,
                                   &drivetrain_output_, &drivetrain_status_);
    drivetrain_motor_plant_.Simulate(&drivetrain_output_);
    //    SimulateTimestep(true);
  }
}

::aos::controls::HPolytope<2> MakeBox(double x1_min, double x1_max,
                                      double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
      /*[*/ -1.0, 0.0 /*]*/,
      /*[*/ 0.0, 1.0 /*]*/,
      /*[*/ 0.0, -1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
      /*[*/ -x1_min /*]*/,
      /*[*/ x2_max /*]*/,
      /*[*/ -x2_min /*]]*/;
  ::aos::controls::HPolytope<2> t_poly(box_H, box_k);
  return t_poly;
}

class CoerceGoalTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// WHOOOHH!
TEST_F(CoerceGoalTest, Inside) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << /*[[*/ 1, -1 /*]]*/;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 1.5, 1.5 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(R(0, 0), output(0, 0));
  EXPECT_EQ(R(1, 0), output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_Intersect) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_no_Intersect) {
  ::aos::controls::HPolytope<2> box = MakeBox(3, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(3.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Middle_Of_Edge) {
  ::aos::controls::HPolytope<2> box = MakeBox(0, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << -1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, PerpendicularLine) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace y2015_bot3
