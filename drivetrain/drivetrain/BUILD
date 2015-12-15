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
    '//drivetrain:state_feedback_loop',
  ],
)

# We'll use the build file to mark when we've taken out dependencies that we need, I suppose.
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
    'commonmath.h', #TODO (Finn): Is this bad practice? It's the only place we need it...
  ],
  deps = [
    ':drivetrain_queue', #Will be removed. Replace with structs.
    '//aos/common/controls:polytope', # Moved because it's used in coerce_goal.
    '//drivetrain:state_feedback_loop',
    '//frc971/control_loops:coerce_goal', #Moved because it's used in testing and we'll want to actually build those tests before we run. TODO (Finn)
    '//frc971/queues:gyro', #Will be removed. Replace with structs.
    '//frc971:shifter_hall_effect', #Will be removed.
  ],
)

# TODO (Finn): This is a 971-ism with the messages sent around.
# We'll want to run the controller in a separate thread and get the messages sent to the main FRC one, eventually, but for now we'll just throw it all in there.
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
