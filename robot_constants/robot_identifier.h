#ifndef ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_
#define ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_

#include <string>

enum class RobotIdentifier {
  COMP = 0,
  APPA,
  SSBB,
  UNSURE
};

RobotIdentifier GetRobotIdentifier();

#endif /* ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_ */
