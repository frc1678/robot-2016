#include "gyro_reader.h"

// This is *NOT* the gyro_reader.cpp from robot-2015.

// Gyro datasheet:
// http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf

GyroReader::GyroReader() : muan::Updateable(100 * hz) {
  gyro = new GyroInterface();
  is_initalized = Init();
}

bool GyroReader::Init() {
  if (!gyro->InitializeGyro()) {
    return false;
  }
  return true;
}

void GyroReader::Calibrate(Time dt) {
  if (deadzone_time > deadzone_time_counter) {
    deadzone_time_counter += dt;
  } else {
    Angle delta_angle = (gyro->ExtractAngle(gyro->GetReading()) * rad / s) * dt;
    if (delta_angle < 2 * deg &&
        delta_angle > -2 * deg) {  // TODO(Wesley) Make not bad.
      calibration_drift_angle += delta_angle;
      calibration_time_counter += dt;
      if (calibration_time_counter >= calibration_time) {
        calibration_drift = calibration_drift_angle / calibration_time_counter;
        is_calibrated = true;
        need_led_switch = true;
      }
    }
  }
}

Angle GyroReader::GetAngle() {
  return angle - offset_;  // That was easy...
}

void GyroReader::SetOffset() {
  offset_ += angle;
}

void GyroReader::Update(Time dt) {
  if (!is_initalized) {
    //printf("Gyro not initalized :'(\n");
  } else {
    if (!is_calibrated) {
      Calibrate(dt);
    } else {
      if (need_led_switch) {
        need_led_switch = false;
      }
      angle += ((gyro->ExtractAngle(gyro->GetReading()) * rad / s -
                 calibration_drift) *
                dt) *
               (360 / degrees_per_circle);
      time += dt;
    }
  }
}

bool GyroReader::IsCalibrated() { return is_calibrated; }
