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
  deps = ["//drivetrain:drivetrain_subsystem"],
  visibility = ["//visibility:public"]
)

cc_binary(
  name="frc1678",
  deps=[
    "@wpilib//:main",
    ":robot_subsystems",
    "//unitscpp",
    "//muan:multithreading",
    "//frc1678/auto",
    "//vision"
  ],
  srcs=glob(["*.cpp"]) #TODO (jasmine): Add button interface as separate target?
)
