# TODO(Kyle): Build libwpi from source instead of using the pre-built binaries
cc_library(
  name = 'main',
  visibility = ['//visibility:public'],
  copts = [
    '-Wno-error',
  ],
  hdrs = glob(['include/**/*.h']),
  includes = ['include'],
  linkopts = [
    '-Lexternal/wpilib/lib',
    '-lwpi'
  ],
)
