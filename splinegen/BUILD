package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'point',
  hdrs = ['point.h'],
  deps = ['//muan/unitscpp']
)

cc_library(
  name = 'curve',
  srcs = ['curve.cpp'],
  hdrs = ['curve.h'],
  deps = ['point']
)

cc_library(
  name = 'hermite',
  srcs = ['hermite_spline.cpp'],
  hdrs = ['hermite_spline.h'],
  deps = ['curve', 'point']
)

cc_library(
  name = 'trajectory',
  srcs = ['reparametrized_trajectory.cpp', 'reparametrize.cpp'],
  hdrs = ['trajectory.h', 'reparametrized_trajectory.h', 'reparametrize.h'],
  deps = ['point', 'curve', 'drive_constraints']
)

cc_library(
  name = 'drive_constraints',
  srcs = ['constraints.cpp'],
  hdrs = ['constraints.h'],
  deps = ['//muan/unitscpp']
)

cc_binary(
  name = 'test',
  srcs = ['test.cpp'],
  deps = ['hermite', 'trajectory']
)
