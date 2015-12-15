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
    '//frc971/control_loops:state_feedback_loop',
  ],
)

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
  ],
  deps = [
    ':drivetrain_queue',
    '//aos/common/controls:control_loop',
    '//aos/common/controls:polytope',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//frc971/queues:gyro',
    '//frc971:shifter_hall_effect',
    '//aos/common/util:log_interval',
    '//aos/common:math',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':drivetrain_lib',
    ':drivetrain_queue',
  ],
)
