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

std::string GetRobotString(RobotIdentifier id);

#endif /* ROBOT_CONSTANTS_ROBOT_IDENTIFIER_H_ */
