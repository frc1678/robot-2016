new_http_archive(
    name = "gtest",
    url = "https://googletest.googlecode.com/files/gtest-1.7.0.zip",
    sha256 = "247ca18dd83f53deb1328be17e4b1be31514cedfc1e3424f672bf11fd7e0d60d",
    build_file = "gtest.BUILD",
)

new_http_archive(
  name = 'arm_frc_linux_gnueabi_repo',
  build_file = 'tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD',
  sha256 = '9e93b0712e90d11895444f720f0c90c649fb9becb4ca28bb50749d9074eb1306',
  url = 'http://frc971.org/Build-Dependencies/roborio-compiler-2016.tar.gz',
)

# Recompressed version of the one downloaded from Linaro at
# <https://releases.linaro.org/15.05/components/toolchain/binaries/arm-linux-gnueabihf/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.xz>,
# with workarounds for <https://github.com/bazelbuild/bazel/issues/574> and the
# top-level folder stripped off.
new_http_archive(
  name = 'linaro_linux_gcc_4.9_repo',
  build_file = 'compilers/linaro_linux_gcc_4.9.BUILD',
  sha256 = '25e97bcb0af4fd7cd626d5bb1b303c7d2cb13acf2474e335e3d431d1a53fbb52',
  url = 'http://frc971.org/Build-Dependencies/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.gz',
)

# Frozen copy of wpilib source
new_http_archive(
  name = 'wpilib',
  build_file = 'wpilib.BUILD',
  sha256 = '367ca2bef14426f04c0c60fd71638009d81fb024e6e42c8c6202746a93e51ab7',
  url = 'https://www.dropbox.com/s/agexr7gt2o838eo/wpilib.zip?dl=0&raw=1'
)

new_http_archive(
  name = 'eigen',
  build_file = 'eigen.BUILD',
  sha256 = 'e37ad303cddf324a16b49e43edfb00129972442c0d8a2d866605bc29de94571a',
  url = 'https://bitbucket.org/eigen/eigen/get/4111270ba6e1.zip'
)
