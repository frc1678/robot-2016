#ifndef FRC1678_ROBOT_PORTS_H_
#define FRC1678_ROBOT_PORTS_H_

namespace RobotPorts {
constexpr unsigned int left_encoder_a = 12, left_encoder_b = 13;
constexpr unsigned int right_encoder_a = 10, right_encoder_b = 11;

constexpr unsigned int drive_left = 2, drive_right = 1;

constexpr unsigned int shift_a = 1, shift_b = 2;
}

namespace JoystickPorts {
constexpr unsigned int throttle = 1, wheel = 0;
constexpr unsigned int shift_low = 2, shift_high = 1, quick_turn = 5;
}

#endif /* ifndef FRC1678_ROBOT_PORTS_H_ */
