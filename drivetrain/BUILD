cc_library(
  name = 'state_feedback_loop',
  hdrs = [
    'state_feedback_loop.h',
  ],
  deps = [
    '//third_party/eigen', #TODO (Finn): How do we include eigen?
    '//aos/common:macros', #remove or add directly?
  ],
)

cc_library(
  name = 'coerce_goal',
  srcs = [
    'coerce_goal.cc', #Do you have me?
  ],
  hdrs = [
    'coerce_goal.h',
  ],
  deps = [
    '//third_party/eigen', #TODO (Finn)
    '//drivetrain:polytope',
    '//debian:libm', # TODO (Finn): Look at 971-Robot-Code/debian/BUILD and figure this out.
  ],
)

cc_library(
  name = 'polytope',
  hdrs = [
    'polytope.h',
  ],
  deps = [
    '//third_party/eigen',
    '//third_party/cddlib', #TODO (jasmine): Add cddlib to our third party stuff.
  ],
)

