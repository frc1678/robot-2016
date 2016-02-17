/* 
Many thanks to 971 Spartan Robotics for their help with this project
*/
#include "drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense" // TODO (Finn): Make sure it actually deals with Eigen.

// #include "aos/common/logging/logging.h" // TODO (Finn): What are we doing with logging?
// For now, remove all references to it, but I'd like to be able to actually log it.
#include "aos_control/polytope.h"
#include "commonmath.h"
//#include "aos/common/logging/queue_logging.h" // This comes in with the logging.
//#include "aos/common/logging/matrix_logging.h"

#include "aos_control/state_feedback_loop.h"
#include "aos_control/coerce_goal.h"
#include "polydrivetrain_cim_plant.h" 
#include "drivetrain_dog_motor_plant.h"
#include "polydrivetrain_dog_motor_plant.h"

// A consistent way to mark code that goes away without shifters.
#define HAVE_SHIFTERS 1

namespace drivetrain {
namespace control_loops {

class PolyDrivetrain {
 public:
  enum Gear { HIGH, LOW, SHIFTING_UP, SHIFTING_DOWN };
  // Stall Torque in N m
  static constexpr double kStallTorque = 2.42;
  // Stall Current in Amps
  static constexpr double kStallCurrent = 133.0;
  // Free Speed in RPM. Used number from last year.
  static constexpr double kFreeSpeed = 4650.0;
  // Free Current in Amps
  static constexpr double kFreeCurrent = 2.7;
  // Moment of inertia of the drivetrain in kg m^2
  // Just borrowed from last year.
  static constexpr double J = 10;
  // Mass of the robot, in kg.
  static constexpr double m = 68;
  // Radius of the robot, in meters (from last year).
  static constexpr double rb = 0.66675 / 2.0;
  static constexpr double kWheelRadius = 0.0515938;
  // Resistance of the motor, divided by the number of motors.
  static constexpr double kR =
      (12.0 / kStallCurrent / 4 + 0.03) / (0.93 * 0.93);
  // Motor velocity constant
  static constexpr double Kv =
      ((kFreeSpeed / 60.0 * 2.0 * M_PI) / (12.0 - kR * kFreeCurrent));
  // Torque constant
  static constexpr double Kt = kStallTorque / kStallCurrent;

  PolyDrivetrain()
      : U_Poly_((Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
                 /*[*/ -1, 0 /*]*/,
                 /*[*/ 0, 1 /*]*/,
                 /*[*/ 0, -1 /*]]*/).finished(),
                (Eigen::Matrix<double, 4, 1>() << /*[[*/ 12 /*]*/,
                 /*[*/ 12 /*]*/,
                 /*[*/ 12 /*]*/,
                 /*[*/ 12 /*]]*/).finished()),
        loop_(new StateFeedbackLoop<2, 2, 2>(
            ::drivetrain::control_loops::MakeVelocityDrivetrainLoop())),
        ttrust_(1.1),
        wheel_(0.0),
        throttle_(0.0),
        quickturn_(false),
        stale_count_(0),
        position_time_delta_(0.01),
        left_gear_(LOW),
        right_gear_(LOW),
        counter_(0) {
  }
  static bool IsInGear(Gear gear) { return gear == LOW || gear == HIGH; }

  static double MotorSpeed(
      bool is_high_gear, double velocity) {
    // TODO(austin): G_high, G_low and kWheelRadius

    if (is_high_gear) {
      return velocity / kDrivetrainHighGearRatio / kWheelRadius;
    } else {
      return velocity / kDrivetrainLowGearRatio / kWheelRadius;
    }
  }

  Gear ComputeGear(double velocity, Gear current) {
    const bool is_high_gear = true; // what we need to pass to MotorSpeed
    const double low_omega = MotorSpeed(!is_high_gear, ::std::abs(velocity));
    const double high_omega =
        MotorSpeed(!is_high_gear, ::std::abs(velocity));

    double high_torque = ((12.0 - high_omega / Kv) * Kt / kR);
    double low_torque = ((12.0 - low_omega / Kv) * Kt / kR);
    double high_power = high_torque * high_omega;
    double low_power = low_torque * low_omega;

    // TODO(aschuh): Do this right!
    if ((current == HIGH || high_power > low_power + 160) &&
        ::std::abs(velocity) > 0.14) {
      return HIGH;
    } else {
      return LOW;
    }
  }

  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear) {
    const double kWheelNonLinearity = 0.5;
    // Apply a sin function that's scaled to make it feel better.
    const double angular_range = M_PI_2 * kWheelNonLinearity;

    quickturn_ = quickturn;
    wheel_ = sin(angular_range * wheel) / sin(angular_range);
    wheel_ = sin(angular_range * wheel_) / sin(angular_range);
    if (!quickturn_) {
      wheel_ *= (.5);
    }

    static const double kThrottleDeadband = 0.05;
    if (::std::abs(throttle) < kThrottleDeadband) {
      throttle_ = 0;
    } else {
      throttle_ = copysign((::std::abs(throttle) - kThrottleDeadband) /
                               (1.0 - kThrottleDeadband),
                           throttle);
    }

    // TODO(austin): Fix the upshift logic to include states.
    Gear requested_gear;
    if (false) {
      const double current_left_velocity =
          (position_.left_encoder - last_position_.left_encoder) /
          position_time_delta_;
      const double current_right_velocity =
          (position_.right_encoder - last_position_.right_encoder) /
          position_time_delta_;

      Gear left_requested = ComputeGear(
                                        current_left_velocity, left_gear_);
      Gear right_requested = ComputeGear(
                                         current_right_velocity, right_gear_);
      requested_gear =
          (left_requested == HIGH || right_requested == HIGH) ? HIGH : LOW;
    } else {
      requested_gear = highgear ? HIGH : LOW;
    }

    const Gear shift_up = kDrivetrainClutchTransmission ? HIGH : SHIFTING_UP;
    const Gear shift_down = kDrivetrainClutchTransmission ? LOW : SHIFTING_DOWN;

    // TODO (jasmine): Work through the shifting logic.
    if (left_gear_ != requested_gear) {
      if (IsInGear(left_gear_)) {
        if (requested_gear == HIGH) {
          left_gear_ = shift_up;
        } else {
          left_gear_ = shift_down;
        }
      } else {
        if (requested_gear == HIGH && left_gear_ == SHIFTING_DOWN) {
          left_gear_ = SHIFTING_UP;
        } else if (requested_gear == LOW && left_gear_ == SHIFTING_UP) {
          left_gear_ = SHIFTING_DOWN;
        }
      }
    }
    if (right_gear_ != requested_gear) {
      if (IsInGear(right_gear_)) {
        if (requested_gear == HIGH) {
          right_gear_ = shift_up;
        } else {
          right_gear_ = shift_down;
        }
      } else {
        if (requested_gear == HIGH && right_gear_ == SHIFTING_DOWN) {
          right_gear_ = SHIFTING_UP;
        } else if (requested_gear == LOW && right_gear_ == SHIFTING_UP) {
          right_gear_ = SHIFTING_DOWN;
        }
      }
    }
  }

  void SetPosition(const DrivetrainPosition *position) {
    if (position == NULL) {
      ++stale_count_;
    } else {
      last_position_ = position_;
      position_ = *position;
      position_time_delta_ = (stale_count_ + 1) * 0.01;
      stale_count_ = 0;
    }

#if HAVE_SHIFTERS
    if (position) {
// Here's one place where we need to just assume that we're shifting.
      // Switch to the correct controller.
      if(!(position->left_shifter_high)) {
         if(!(position->right_shifter_high)) {
           // Both are low.
           loop_->set_controller_index(0);
         } else {
           // Left low, right high.
           loop_->set_controller_index(1);
         }
       } else { // Left high.
         if(!(position->right_shifter_high)) {
           // Both are low.
           loop_->set_controller_index(2);
         } else {
           // Left low, right high.
           loop_->set_controller_index(3);
         }
       }
     }
#endif
  }

  double FilterVelocity(double throttle) {
    const Eigen::Matrix<double, 2, 2> FF =
        loop_->B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

    constexpr int kHighGearController = 3;
    const Eigen::Matrix<double, 2, 2> FF_high =
        loop_->controller(kHighGearController).plant.B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() -
         loop_->controller(kHighGearController).plant.A());

    ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
    int min_FF_sum_index;
    const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
    const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
    const double high_min_FF_sum = FF_high.col(0).sum();

    const double adjusted_ff_voltage = ::aos::Clip(
        throttle * 12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
    return (adjusted_ff_voltage +
            ttrust_ * min_K_sum * (loop_->X_hat(0, 0) + loop_->X_hat(1, 0)) /
                2.0) /
           (ttrust_ * min_K_sum + min_FF_sum);
  }

  double MaxVelocity() {
    const Eigen::Matrix<double, 2, 2> FF =
        loop_->B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

    constexpr int kHighGearController = 3;
    const Eigen::Matrix<double, 2, 2> FF_high =
        loop_->controller(kHighGearController).plant.B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() -
         loop_->controller(kHighGearController).plant.A());

    ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
    int min_FF_sum_index;
    const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
    // const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
    const double high_min_FF_sum = FF_high.col(0).sum();

    const double adjusted_ff_voltage =
        ::aos::Clip(12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
    return adjusted_ff_voltage / min_FF_sum;
  }

  void Update() {
    // TODO(austin): Observer for the current velocity instead of difference
    // calculations.
    ++counter_;
#if HAVE_SHIFTERS
    const double current_left_velocity =
        (position_.left_encoder - last_position_.left_encoder) /
        position_time_delta_;
    const double current_right_velocity =
        (position_.right_encoder - last_position_.right_encoder) /
        position_time_delta_;
    const double left_motor_speed =
        MotorSpeed(position_.left_shifter_high,
                   current_left_velocity);
    const double right_motor_speed =
        MotorSpeed(position_.right_shifter_high,
                   current_right_velocity);

#endif

#if HAVE_SHIFTERS
    if (IsInGear(left_gear_) && IsInGear(right_gear_)) {
#else
    {
#endif
      // FF * X = U (steady state)
      const Eigen::Matrix<double, 2, 2> FF =
          loop_->B().inverse() *
          (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

      // Invert the plant to figure out how the velocity filter would have to
      // work
      // out in order to filter out the forwards negative inertia.
      // This math assumes that the left and right power and velocity are
      // equals,
      // and that the plant is the same on the left and right.
      const double fvel = FilterVelocity(throttle_);

      const double sign_svel = wheel_ * ((fvel > 0.0) ? 1.0 : -1.0);
      double steering_velocity;
      if (quickturn_) {
        steering_velocity = wheel_ * MaxVelocity() * 2;//Five is constant to scale up sensitivity of quickturn wheel turning.
      } else {
        steering_velocity = ::std::abs(fvel) * wheel_;
      }
      const double left_velocity = fvel - steering_velocity;
      const double right_velocity = fvel + steering_velocity;
      //LOG(DEBUG, "l=%f r=%f\n", left_velocity, right_velocity);

      // Integrate velocity to get the position.
      // This position is used to get integral control.
      loop_->mutable_R() << left_velocity, right_velocity;

      if (!quickturn_) {
        // K * R = w
        Eigen::Matrix<double, 1, 2> equality_k;
        equality_k << 1 + sign_svel, -(1 - sign_svel);
        const double equality_w = 0.0;

        // Construct a constraint on R by manipulating the constraint on U
        ::aos::controls::HPolytope<2> R_poly = ::aos::controls::HPolytope<2>(
            U_Poly_.H() * (loop_->K() + FF),
            U_Poly_.k() + U_Poly_.H() * loop_->K() * loop_->X_hat());

        // Limit R back inside the box.
        loop_->mutable_R() = frc971::control_loops::CoerceGoal(
            R_poly, equality_k, equality_w, loop_->R());
      }

      const Eigen::Matrix<double, 2, 1> FF_volts = FF * loop_->R();
      const Eigen::Matrix<double, 2, 1> U_ideal =
          loop_->K() * (loop_->R() - loop_->X_hat()) + FF_volts;

      for (int i = 0; i < 2; i++) {
        loop_->mutable_U()[i] = ::aos::Clip(U_ideal[i], -12, 12);
      }

      // TODO(austin): Model this better.
      // TODO(austin): Feed back?
      loop_->mutable_X_hat() =
          loop_->A() * loop_->X_hat() + loop_->B() * loop_->U();
#if HAVE_SHIFTERS
    } else {
      // Any motor is not in gear.  Speed match.
      ::Eigen::Matrix<double, 1, 1> R_left;
      ::Eigen::Matrix<double, 1, 1> R_right;
      R_left(0, 0) = left_motor_speed;
      R_right(0, 0) = right_motor_speed;

      const double wiggle =
          (static_cast<double>((counter_ % 20) / 10) - 0.5) * 5.0;

      loop_->mutable_U(0, 0) =
          ::aos::Clip((R_left / Kv)(0, 0) + (IsInGear(left_gear_) ? 0 : wiggle),
                      -12.0, 12.0);
      loop_->mutable_U(1, 0) = ::aos::Clip(
          (R_right / Kv)(0, 0) + (IsInGear(right_gear_) ? 0 : wiggle), -12.0,
          12.0);
      loop_->mutable_U() *= 12.0 / 12.0;// ::aos::robot_state->voltage_battery;
      // TODO (jasmine): Don't assume that we're at 12 volts.
#endif
    }
  }

  void SendMotors(DrivetrainOutput *output) {
    if (output != NULL) {
      output->left_voltage = loop_->U(0, 0);
      output->right_voltage = loop_->U(1, 0);
      output->left_high = left_gear_ == HIGH || left_gear_ == SHIFTING_UP;
      output->right_high = right_gear_ == HIGH || right_gear_ == SHIFTING_UP;
    }
  }

 private:
  const ::aos::controls::HPolytope<2> U_Poly_;

  ::std::unique_ptr<StateFeedbackLoop<2, 2, 2>> loop_;

  const double ttrust_;
  double wheel_;
  double throttle_;
  bool quickturn_;
  int stale_count_;
  double position_time_delta_;
  Gear left_gear_;
  Gear right_gear_;
  DrivetrainPosition last_position_;
  DrivetrainPosition position_;
  int counter_;
};
constexpr double PolyDrivetrain::kStallTorque;
constexpr double PolyDrivetrain::kStallCurrent;
constexpr double PolyDrivetrain::kFreeSpeed;
constexpr double PolyDrivetrain::kFreeCurrent;
constexpr double PolyDrivetrain::J;
constexpr double PolyDrivetrain::m;
constexpr double PolyDrivetrain::rb;
constexpr double PolyDrivetrain::kWheelRadius;
constexpr double PolyDrivetrain::kR;
constexpr double PolyDrivetrain::Kv;
constexpr double PolyDrivetrain::Kt;

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
// This will be important once we get it running in its own thread
// but for now, let's just throw it directly into the main loop
// to see what happens.

void DrivetrainLoop::RunIteration(const DrivetrainGoal *goal,
                                  const DrivetrainPosition *position,
                                  DrivetrainOutput *output) {
  static PolyDrivetrain dt_openloop;

  double wheel = goal->steering;
  double throttle = goal->throttle;
  bool quickturn = goal->quickturn;
  bool highgear = goal->highgear;

  dt_openloop.SetGoal(wheel, throttle, quickturn, highgear);

  dt_openloop.SetPosition(position);
  dt_openloop.Update();
  dt_openloop.SendMotors(output);
}

}  // namespace control_loops
}  // namespace drivetrain
