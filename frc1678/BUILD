cc_library(
  name = "robot_ports",
  hdrs = [
    "robot_ports.h"
  ],
  visibility = ["//visibility:public"]
)

cc_binary(
  name="frc1678",
  deps=[
    "@wpilib//:main",
    "//drivetrain:drivetrain_subsystem",
    "//unitscpp",
    "//muan:multithreading",
    "//vision"
  ],
  srcs=glob(["*.cpp", "*.h"]) #TODO (jasmine): Add button interface as separate target?
)
