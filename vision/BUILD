cc_library(
  name = "robot_side",
  srcs = ["vision.cpp"],
  hdrs = ["vision.h"],
  deps = ["@wpilib//:main", "//frc1678:robot_subsystems"]
)

cc_library(
  name = "object_detector",
  srcs = ["object_detector.cpp", "shape_detector.cpp", "in_range_instructions.cpp"],
  hdrs = ["object_detector.h", "shape_detector.h", "in_range_instructions.h"],
  deps = ["@opencv//:main", "//unitscpp"]
)

cc_library(
  name = "vision_sender",
  srcs = ["data_sender.cpp"],
  hdrs = ["data_sender.h"],
  deps = [":object_detector", "//unitscpp", "//muan", "@opencv//:main"]
)

cc_binary(
  name = "coprocessor_side",
  srcs = ["vision_main.cpp"],
  deps = ["object_detector", "vision_sender", "//unitscpp", "//muan", ":vision_utils", "@opencv//:main"],
  linkopts = ["-lpthread"]
)

cc_library(
  name = "vision_utils",
  srcs = ["vision_utils.cpp"],
  hdrs = ["vision_utils.h"],
  deps = ["@opencv//:main"]
)
