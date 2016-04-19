#ifndef FRC1678_ROBOT_PORTS_H_
#define FRC1678_ROBOT_PORTS_H_

namespace RobotPorts {
constexpr unsigned int left_encoder_a = 12, left_encoder_b = 13;
constexpr unsigned int right_encoder_a = 10, right_encoder_b = 11;

constexpr unsigned int drive_left = 1, drive_right = 0;

constexpr unsigned int shift = 7;

// Arm Ports
constexpr unsigned int elevator_motor_a = 4, elevator_motor_b = 5;
constexpr unsigned int shooter_a = 7, shooter_b = 70;
constexpr unsigned int pivot_motor_a = 2, pivot_motor_b = 3;
constexpr unsigned int intake_side = 6, intake_front = 8;

constexpr unsigned int pivot_encoder_a = 14, pivot_encoder_b = 15;
constexpr unsigned int shooter_encoder_a = 16,
                       shooter_encoder_b = 17;  // Unknown
constexpr unsigned int elevator_encoder_a = 18,
                       elevator_encoder_b = 19;  // Unknown

constexpr unsigned int ball_sensor = 1;

// Hall effect sensors for the pivot, none onthe elevator
constexpr unsigned int pivot_hall = 0;
constexpr unsigned int intake_proxy_port = 22;

// Solenoid ports for the disk brakes on the pivot and the elevator
constexpr unsigned int pivot_brake_a = 0, pivot_brake_b = 1;
constexpr unsigned int elevator_brake_a = 2, elevator_brake_b = 3;

// Solenoid port for shooter hood
constexpr unsigned int shooter_hood = 6;

constexpr unsigned int ball_pinch = 4;
}

namespace JoystickPorts {
constexpr unsigned int throttle = 1, wheel = 0;
constexpr unsigned int shift_low = 2, shift_high = 1, quick_turn = 5;
constexpr unsigned int prep = 1, shoot = 2;
}

#endif /* ifndef FRC1678_ROBOT_PORTS_H_ */
