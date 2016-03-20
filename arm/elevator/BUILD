cc_library(
  name = "elevator_controller", 
  srcs = ["elevator_controller.cpp", "elevator_constants.h"],
  hdrs = ["elevator_controller.h"],
  deps = ["//muan:control", "//muan/unitscpp", "@eigen//:main", "//muan:utils", "//muan:logging", "//robot_constants"],
  visibility = ["//visibility:public"]
)

cc_test(
  name = "elevator_test",
  srcs = ["elevator_test.cpp"],
  deps = [":elevator_controller", "@gtest//:main"]
)
