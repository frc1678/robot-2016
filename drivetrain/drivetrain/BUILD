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
    'commonmath.h', #TODO (Finn): Is this bad practice? It's the only place we need it...
  ],
  deps = [
    '//drivetrain:polytope',
    '//drivetrain:state_feedback_loop',
    '//drivetrain:coerce_goal',
  ],
  visibility = ["//visibility:public"],
)

# TODO (Finn): This is a 971-ism with the messages sent around.
# We'll want to run the controller in a separate thread and get the messages sent to the main FRC one, eventually, but for now we'll just throw it all in there.
