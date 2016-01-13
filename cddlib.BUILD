cc_library(
  name = 'main',
  visibility = ['//visibility:public'],
  srcs = [
    'lib-src/cddcore.c',
    'lib-src/cddlp.c',
    'lib-src/cddmp.c',
    'lib-src/cddio.c',
    'lib-src/cddlib.c',
    'lib-src/cddproj.c',
    'lib-src/setoper.c',
    'lib-src/cddmp.h',
    'lib-src/cddtypes.h',
    'lib-src/setoper.h',
  ],
  includes = [
    'lib-src'
  ],
  hdrs = [
    'lib-src/cdd.h',
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-switch-enum',
    '-Wno-empty-body',
    '-Wno-sign-compare',
    '-Wno-unused-result',
    '-Wno-unused-but-set-variable',
    '-Wno-format-extra-args',
    '-Wno-maybe-uninitialized',
    '-Wno-discarded-qualifiers'
  ]
)
