new_http_archive(
    name = "gtest",
    url = "https://googletest.googlecode.com/files/gtest-1.7.0.zip",
    sha256 = "247ca18dd83f53deb1328be17e4b1be31514cedfc1e3424f672bf11fd7e0d60d",
    build_file = "gtest.BUILD",
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

new_http_archive(
  name = 'opencv',
  build_file = 'opencv.BUILD',
  sha256 = '5d6d80c0d1052cfbbb074df774b721f482b6cd63566151ad09ca0f8f459a6aaa',
  url = 'https://www.dropbox.com/s/t00zz2wtqmabum9/opencv.zip?dl=0&raw=1'
)

new_git_repository(
  name = 'cddlib',
  build_file = 'cddlib.BUILD',
  remote = 'https://github.com/mcmtroffaes/cddlib',
  commit = '370919d'
)
