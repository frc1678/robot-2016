cc_binary(
  name = "main",
  srcs = ["vision_main.cpp"],
  deps = ["object_detector", "vision_sender", "//unitscpp", "//muan", ":vision_utils", "@opencv//:main"],
  linkopts = ["-lpthread"]
)

cc_binary(
  name = "calibration",
  srcs = ["calibration_main.cpp"],
  deps = ["@opencv//:main", ":image_calibration", ":vision_utils"]
)

cc_library(
  name = "object_detector",
  srcs = ["object_detector.cpp", "shape_detector.cpp"],
  hdrs = ["object_detector.h", "shape_detector.h"],
  deps = ["@opencv//:main", "//unitscpp", ":vision_utils", '//muan:utils']
)

cc_library(
  name = "vision_sender",
  srcs = ["data_sender.cpp"],
  hdrs = ["data_sender.h"],
  deps = [":object_detector", "//unitscpp", "//muan", "@opencv//:main", "//citrus_socket"]
)

cc_library(
  name = "vision_utils",
  srcs = ["vision_utils.cpp", "maximization.cpp", "in_range_instructions.cpp"],
  hdrs = ["vision_utils.h", "maximization.h", "in_range_instructions.h"],
  deps = ["@opencv//:main", '//muan/unitscpp']
)

cc_library(
  name = "image_calibration",
  srcs = ["calibration.cpp", "score_range_vals.cpp", "img_score_calculator.cpp", "score_rules.cpp"],
  hdrs = ["calibration.h", "score_range_vals.h", "img_score_calculator.h", "score_rules.h"],
  deps = ["@opencv//:main", ":vision_utils"]
)

cc_test(
  name = "tests",
  srcs = ["tests.cpp"],
  deps = ["@opencv//:main", "@gtest//:main", ":object_detector", ":vision_sender", ":vision_utils", "//muan", "//unitscpp"]
)
