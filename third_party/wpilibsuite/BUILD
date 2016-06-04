licenses(['permissive'])
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'ntcore',
  srcs = glob(['ntcore/src/**/*.cpp', 'ntcore/src/**/*.h']),
  hdrs = glob(['ntcore/include/**/*.h']),
  includes = ['ntcore/include']
)

cc_library(
  name = 'ni_libraries',
  srcs = glob(['allwpilib/ni-libraries/*so*'], exclude = [
    'allwpilib/ni-libraries/libwpi_*.so',
    'allwpilib/ni-libraries/libwpi.so'
  ]),
  visibility = ['//visibility:public'],
  linkstatic = 1
)

cc_library(
  name = 'hal',
  srcs = glob([
    'allwpilib/hal/lib/**/*.cpp', 'allwpilib/hal/lib/**/*.h'
  ]),
  hdrs = glob([
    'allwpilib/hal/include/**/*.h',
    'allwpilib/hal/include/**/*.hpp'
  ]),
  includes = [
    'allwpilib/hal/include',
    'allwpilib/hal/lib/athena/FRC_FPGA_ChipObject',
    'allwpilib/hal/lib/athena'
  ],
  linkopts = [
    '-lpthread'
  ],
  deps = [':ni_libraries'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'wpilib',
  srcs = glob([
    'allwpilib/wpilibc/athena/src/**/*.cpp',
    'allwpilib/wpilibc/shared/src/**/*.cpp'
  ]),
  hdrs = glob([
    'allwpilib/wpilibc/athena/include/**/*.h',
    'allwpilib/wpilibc/athena/include/**/*.hpp',
    'allwpilib/wpilibc/shared/include/**/*.h',
    'allwpilib/wpilibc/shared/include/**/*.hpp',
    'allwpilib/wpilibc/shared/include/**/*.inc'
  ]),
  includes = [
    'allwpilib/wpilibc/athena/include',
    'allwpilib/wpilibc/shared/include'
  ],
  linkopts = [
    '-lpthread',
    '-ldl'
  ],
  deps = [
    ':hal',
    ':ntcore'
  ]
)
