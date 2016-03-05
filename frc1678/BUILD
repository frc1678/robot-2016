cc_library(
  name = "robot_ports",
  hdrs = [
    "robot_ports.h"
  ],
  visibility = ["//visibility:public"]
)

cc_library(
  name = "robot_subsystems",
  hdrs = ["robot_subsystems.h"],
  deps = ["//drivetrain:drivetrain_subsystem", "//arm:arm_subsystem"],
  visibility = ["//visibility:public"]
)

cc_library(
  name = "declarations",
  hdrs = ["frc1678.h"],
  deps = [":robot_subsystems", "//vision"],
  visibility = ["//visibility:public"]
)

cc_binary(
  name="frc1678",
  deps=[
    "@wpilib//:main",
    ":robot_subsystems",
    "//muan/unitscpp",
    "//muan:multithreading",
    "//frc1678/auto",
    "//vision",
    "//robot_constants",
    "//utils:citrus_button"
  ],
  srcs=glob(["*.cpp"])
)
