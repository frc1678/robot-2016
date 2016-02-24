#include "robot_identifier.h"
#include <mutex>
#include <fstream>
#include <iostream>

RobotIdentifier FindRobotIdentifier() {
  std::ifstream file("/sys/class/net/eth0/address");
  std::string mac;
  std::getline(file, mac);
  RobotIdentifier id;
  // TODO(Kyle) Put the MAC address for PBOT-1 in here when we get the robot
  // back
  if (mac == "00:80:2f:17:fa:f4") {
    id = RobotIdentifier::PRACTICE_ROBOT_2;
  } else if (mac == "00:80:2f:21:a9:33") {
    id = RobotIdentifier::COMPETITION_ROBOT;
  } else {
    id = RobotIdentifier::UNSURE;
  }
  return id;
}

RobotIdentifier GetRobotIdentifier() {
  static RobotIdentifier id_;
  static std::once_flag flag_;
  std::call_once(flag_, []() { id_ = FindRobotIdentifier(); });
  return id_;
}
