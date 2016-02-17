cc_library(
  name = "robot_ports",
  hdrs = [
    "robot_ports.h"
  ],
  visibility = ["//visibility:public"]
)

cc_library(
  name = 'robot_subsystems',
  hdrs = [
    'robot_subsystems.h'
  ],
  deps = [
    '//drivetrain:drivetrain_subsystem',
    '//arm:arm_subsystem',
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'citrus_button',
  srcs = [
    'CitrusButton.cpp',
  ],
  hdrs = [
    'CitrusButton.h',
  ],
  deps = [
    '@wpilib//:main',
  ],
)

cc_library(
  name = 'frc1678_interface',
  srcs = [
  ],
  deps = [
    ':citrus_button',
    '@wpilib//:main',
  ],
)

cc_binary(
  name = 'frc1678',
  srcs = [
    'frc1678.cpp',
    'frc1678.h',
    'auto/auto_functions.cpp',
    'auto/auto_routines.cpp',
    'auto/auto_functions.h',
    'auto/auto_routines.h',
  ],
  deps = [
    ':citrus_button',
    ':frc1678_interface',
    ':robot_subsystems',
    '//muan:multithreading',
    '//muan:utils',
    '//unitscpp',
    '//vision',
    '@wpilib//:main',
    '//lemonscript',
    '//gyro',
  ],
)
