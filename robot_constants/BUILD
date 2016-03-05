cc_library(
  name = "robot_identifier",
  srcs = ["robot_identifier.cpp"],
  hdrs = ["robot_identifier.h"],
  visibility = ["//visibility:public"]
)

cc_library(
  name = "robot_constants",
  srcs = ["robot_constants.cpp"],
  hdrs = ["robot_constants.h"],
  visibility = ["//visibility:public"],
  deps = ["//muan", ":robot_identifier"]
)
