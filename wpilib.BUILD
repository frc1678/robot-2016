# TODO(Kyle): Build libwpi from source instead of using the pre-built binaries
cc_library(
  name = 'main',
  visibility = ['//visibility:public'],
  copts = [
    '-Wno-error',
  ],
  srcs = [
    'lib/libwpilib_nonshared.a',
    'lib/libntcore.a',
    'lib/libHALAthena.a',
    'lib/libFRC_NetworkCommunication.so.16.0.0',
    'lib/libi2c.so.1.0.0',
    'lib/libni_emb.so.8.0.0',
    'lib/libNiFpgaLv.so.15.0.0',
    'lib/libNiFpga.so.15.0.0',
    'lib/libnirio_emb_can.so.15.0.0',
    'lib/libNiRioSrv.so.15.0.0',
    'lib/libni_rtlog.so.2.4.0',
    'lib/libRoboRIO_FRC_ChipObject.so.16.0.0',
    'lib/libspi.so.1.0.0',
    'lib/libvisa.so',
  ],
  hdrs = glob([
    'include/**/*.h',
    'include/**/*.hpp',
    'include/**/*.inc',
  ]),
  includes = ['include'],
  linkstatic = 1,
)
