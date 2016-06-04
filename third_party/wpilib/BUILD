licenses(['permissive'])

cc_library(
  name = 'wpilib',
  srcs = glob(['wpilibc/Athena/src/**/*.cpp', 'wpilibc/shared/src/**/*.cpp']),
  hdrs = glob([
    'wpilibc/Athena/include/**/*.h',
    'wpilibc/Athena/include/**/*.hpp',
    'wpilibc/shared/include/**/*.h',
    'wpilibc/shared/include/**/*.hpp',
    'wpilibc/shared/include/**/*.inc'
  ]),
  includes = ['wpilibc/Athena/include', 'wpilibc/shared/include'],
  linkopts = [
    '-lpthread',
    '-ldl'
  ],
  deps = ['//third_party/wpilib/hal', '//third_party/wpilib/ntcore'],
  visibility = ['//visibility:public'],
)
