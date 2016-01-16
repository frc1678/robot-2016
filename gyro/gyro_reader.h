#ifndef DRIVECONTROLLER_H_
#define DRIVECONTROLLER_H_

#include <WPILib.h>
#include "gyro_interface.h"
#include "unitscpp/unitscpp.h"
#include "muan/multithreading/updateable.h"

class GyroReader : public muan::Updateable {
 protected:
  GyroInterface *gyro;

  // This is the number of degrees that the gyro reports per full circle.
  // Calculate this by spinning the robot in circles a large number of times.
  // ( I used 80 times to get the number 355.12375 ), then dividing the number
  // of degrees that the gyro reports by the number of times that you spun it.
  // This should probably be done once per gyro, but more testing is needed to
  // determine if this is the case.
  //
  // You should also check that you get a similar number for smaller numbers of
  // turns.  For example, to get the constant 355.12375, the following values
  // were used:
  //
  // 50 turns, 17715.0 degrees = 354.3 deg/circle
  // 50 turns, 17683.7 degrees = 353.674 deg/circle
  // 80 turns, 28409.9 degrees = 355.12375 deg/circle
  //
  // Only use the value from the measurement with the largest number of turns,
  // as it will be most accurate.
  const float degrees_per_circle = 355.12375;

  // State variables
  bool is_initalized = false;
  bool is_calibrated = false;
  bool need_led_switch = false;

  const Time deadzone_time = 15 * s;
  Time deadzone_time_counter = 0 * s;

  const Time calibration_time = 30 * s;
  Time calibration_time_counter = 0 * s;

  Time time = 0 * s;

  AngularVelocity calibration_drift = 0 * rad / s;
  Angle calibration_drift_angle = 0 * rad;

  Angle angle = 0 * rad;

  DigitalOutput *dio;

  int trial = 1;

 public:
  GyroReader();
  bool Init();
  void Calibrate(Time dt);
  Angle GetAngle();
  virtual void Update(Time dt);
};

#endif
