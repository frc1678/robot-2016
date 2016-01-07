#include "gyro_reader.h"

// This is *NOT* the gyro_reader.cpp from robot-2015.

// Gyro datasheet:
// http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf

GyroReader::GyroReader() : Updateable(100*hz) {
  gyro = new GyroInterface();
  dio = new DigitalOutput(1);
  dio->Set(1);
  is_initalized = Init();
}

bool GyroReader::Init() {
  if (!gyro->InitializeGyro()) {
    return false;
    printf("Gyro init returned false");
  }
  return true;
}

void GyroReader::Calibrate(Time dt) {
  if (deadzone_time > deadzone_time_counter) {
    deadzone_time_counter += dt;
    Angle delta_angle = (gyro->ExtractAngle(gyro->GetReading())*rad/s) * dt; // Do I need this?
  } else {
    Angle delta_angle = (gyro->ExtractAngle(gyro->GetReading())*rad/s) * dt;
    if (delta_angle < 2*deg && delta_angle > -2*deg) { // TODO(Wesley) Make not bad.
      calibration_drift_angle += delta_angle;
      calibration_time_counter += dt;
      //printf("%f\t%f\n", calibration_time_counter.to(s), delta_angle.to(deg));
      if (calibration_time_counter >= calibration_time) {
        calibration_drift = calibration_drift_angle / calibration_time_counter;
        //printf("Final calib val: %f\n", calibration_drift.to(deg/s));
        //printf("#--- START TRIAL %d ---\n", trial);
        is_calibrated = true;
        need_led_switch = true;
      }
    }
  }
}

Angle GyroReader::GetAngle() {
  return angle; // That was easy...
}

void GyroReader::Update(Time dt) {
  if (!is_initalized) {
    printf("Gyro not initalized :'(\n");
  } else {
    if (!is_calibrated) {
      Calibrate(dt);
    } else {
      if (need_led_switch) {
        dio->Set(0);
        need_led_switch = false;
      }
      angle += ((gyro->ExtractAngle(gyro->GetReading())*rad/s - calibration_drift) * dt) * (360/degrees_per_circle);
      time += dt;
      //printf("%f\t%f\n", time.to(s),  angle.to(deg));
      //if (time >= 140*s) {
        //time = 0*s;
        //angle = 0*rad;
        //is_calibrated = false;
        //calibration_time_counter = 0*s;
        //calibration_drift_angle = 0*rad;
        //deadzone_time_counter = 0*s;
        //trial++;
      //}
    }
  }
}
