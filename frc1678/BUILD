cc_binary(
  name="frc1678",
  deps=[
    "@wpilib//:main",
    "//drivetrain/drivetrain:drivetrain_lib",
    "//unitscpp",
    "//muan:multithreading"
  ],
  srcs=glob(["*.cpp", "CitrusButton.h"]) #TODO (jasmine): Add button interface as separate target?
)
