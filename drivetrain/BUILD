cc_library(
  name = 'polydrivetrain_plants',
  srcs = [
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
  ],
  hdrs = [
    'polydrivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.h',
  ],
  deps = [
    '//aos_control:state_feedback_loop',
  ],
)

#TODO (Finn): Maybe move things into the outer drivetrain folder? Will require redoing all the includes.
cc_library(
  name = 'drivetrain_lib',
  srcs = [
    'drivetrain.cc',
    'polydrivetrain_cim_plant.cc',
    'drivetrain_dog_motor_plant.cc',
    'polydrivetrain_dog_motor_plant.cc',
  ],
  hdrs = [
    'drivetrain.h',
    'polydrivetrain_cim_plant.h',
    'drivetrain_dog_motor_plant.h',
    'polydrivetrain_dog_motor_plant.h',
    'commonmath.h',
  ],
  deps = [
    '//aos_control:polytope',
    '//aos_control:state_feedback_loop',
    '//aos_control:coerce_goal',
  ],
  visibility = ["//visibility:public"],
)

cc_test(
  name='drivetrain_lib_test',
  srcs = [
    'drivetrain_lib_test.cc'
  ],
  deps = [
    '//aos_control:state_feedback_loop',
    '//aos_control:coerce_goal',  
    'drivetrain_lib',
    '@gtest//:main',
  ],
  visibility = ["//visibility:public"],
)

cc_library(
  name = 'drivetrain_subsystem',
  srcs = ['drivetrain_subsystem.cpp'],
  hdrs = ['drivetrain_subsystem.h'],
  deps = ['drivetrain_lib', '//muan', '//frc1678:robot_ports', '@wpilib//:main', '//gyro',  '//utils:smart_dashboard_helper', '//muan/unitscpp'],
  visibility = ["//visibility:public"]
)
