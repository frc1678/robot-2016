#include "auto_routines.h"
#include "auto_functions.h"
#include "lemonscript/lemonscript/AvailableCppCommandDeclaration.h"
#include <vector>
#include <iostream>

LemonScriptRunner::LemonScriptRunner(const std::string &auto_routine_file,
                                     CitrusRobot *robot) {
  state.userData = reinterpret_cast<int *>(
      robot);  // Whatever goes in userData comes out later.

  using lemonscript::AvailableCppCommandDeclaration;
  using lemonscript::DataType;

  AvailableCppCommandDeclaration *driveStraight =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraight, "DriveStraight", {DataType::FLOAT, DataType::BOOLEAN});

  AvailableCppCommandDeclaration *driveSlowStraight =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveSlowStraight, "DriveSlowStraight", {DataType::FLOAT, DataType::BOOLEAN});

  AvailableCppCommandDeclaration *driveYolo =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveYolo, "DriveYolo", {DataType::FLOAT, DataType::BOOLEAN});

  AvailableCppCommandDeclaration *driveYoloAtAngle =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveYoloAtAngle, "DriveYoloAtAngle", {DataType::FLOAT, DataType::FLOAT, DataType::BOOLEAN});

  AvailableCppCommandDeclaration *driveStraightAtAngle =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraightAtAngle, "DriveStraightAtAngle", {DataType::FLOAT, DataType::FLOAT, DataType::BOOLEAN});

  AvailableCppCommandDeclaration *pointTurn =
      new AvailableCppCommandDeclaration((void *)AutoFunction::PointTurn, "PointTurn", {DataType::FLOAT});

  AvailableCppCommandDeclaration *absolutePointTurn =
      new AvailableCppCommandDeclaration((void *)AutoFunction::AbsolutePointTurn, "AbsolutePointTurn", {DataType::FLOAT});

  AvailableCppCommandDeclaration *shift =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Wait, "Shift", {DataType::BOOLEAN});

  AvailableCppCommandDeclaration *wait =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Wait, "Wait", {DataType::FLOAT});

  AvailableCppCommandDeclaration *encoderWait =
      new AvailableCppCommandDeclaration((void *)AutoFunction::EncoderWait, "EncoderWait", {DataType::FLOAT});

  AvailableCppCommandDeclaration *shoot = 
          new AvailableCppCommandDeclaration((void *)AutoFunction::Shoot, "Shoot", {});

  AvailableCppCommandDeclaration *waitForBall = 
          new AvailableCppCommandDeclaration((void *)AutoFunction::WaitForBall, "WaitForBall", {});

  AvailableCppCommandDeclaration *runIntake = 
          new AvailableCppCommandDeclaration((void *)AutoFunction::RunIntake, "RunIntake", {DataType::INT});

  AvailableCppCommandDeclaration *setArmPosition = 
          new AvailableCppCommandDeclaration((void *)AutoFunction::SetArmPosition, "SetArmPosition", {DataType::INT});

  AvailableCppCommandDeclaration *checkArmCalibration = 
          new AvailableCppCommandDeclaration((void *)AutoFunction::CheckArmCalibration, "CheckArmCalibration", {});

  AvailableCppCommandDeclaration *align =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Align, "Align", {});

  AvailableCppCommandDeclaration *dropBall =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DropBall, "DropBall", {});

  AvailableCppCommandDeclaration *stopDriving =
      new AvailableCppCommandDeclaration((void *)AutoFunction::StopDriving, "StopDriving", {});

  AvailableCppCommandDeclaration *setWedge =
      new AvailableCppCommandDeclaration((void *)AutoFunction::SetWedge, "SetWedge", {DataType::BOOLEAN});
  

  std::vector<const AvailableCppCommandDeclaration *> commands = {
      driveStraight, driveSlowStraight, driveYolo, driveYoloAtAngle, driveStraightAtAngle, pointTurn, absolutePointTurn, wait, encoderWait, shoot, runIntake, setArmPosition, checkArmCalibration, align, shift, setWedge, waitForBall, dropBall, stopDriving};
  state.declareAvailableCppCommands(commands);

  try {
    compiler = new lemonscript::LemonScriptCompiler(auto_routine_file, &state);

  } catch (std::string error) {
    compiler = NULL;
    std::cerr << error << std::endl;
  }
}

LemonScriptRunner::~LemonScriptRunner() { delete compiler; }

void LemonScriptRunner::Update() { 
        if(compiler) {
                compiler->PeriodicUpdate();
        }
}
