#ifndef ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_
#define ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_

#include <string>

enum class RobotIdentifier {
  COMPETITION_ROBOT = 0,
  PRACTICE_ROBOT_1,
  PRACTICE_ROBOT_2,
  UNSURE
};

RobotIdentifier GetRobotIdentifier();

#endif /* ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_ */
