licenses(['permissive'])

cc_library(
  name = 'hal',
  srcs = glob(['lib/**/*.cpp', 'lib/**/*.h']),
  hdrs = glob(['include/**/*.h', 'include/**/*.hpp']),
  includes = [
    'include',
    'lib/Athena/FRC_FPGA_ChipObject',
    'lib/Athena'
  ],
  linkopts = [
    '-lpthread'
  ],
  deps = ['//third_party/wpilib/ni_libraries'],
  visibility = ['//visibility:public']
)
