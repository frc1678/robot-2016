cc_library(
  name = 'state_feedback_loop',
  hdrs = [
    'state_feedback_loop.h',
  ],
  deps = [
    '//tools/eigen',
    '//aos/common:macros', #remove or add directly?
  ],
)

cc_library(
  name = 'coerce_goal',
  srcs = [
    'coerce_goal.cc',
  ],
  hdrs = [
    'coerce_goal.h',
  ],
  deps = [
    '//tools/eigen',
    '//drivetrain:polytope',
#    '//debian:libm', # TODO (Finn): Look at 971-Robot-Code/debian/BUILD and figure this out.
  ],
)

cc_library(
  name = 'polytope',
  hdrs = [
    'polytope.h',
  ],
  deps = [
    '//tools/eigen',
    '//tools/cddlib', #TODO (jasmine): Add cddlib to our third party stuff.
  ],
)

