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

  AvailableCppCommandDeclaration *driveStraight =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraight, "DriveStraight", {FLOAT});
  
  AvailableCppCommandDeclaration *driveStraightAtAngle =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraight, "DriveStraightAtAngle", {FLOAT, FLOAT});

  AvailableCppCommandDeclaration *pointTurn =
      new AvailableCppCommandDeclaration((void *)AutoFunction::PointTurn, "PointTurn", {FLOAT});

  AvailableCppCommandDeclaration *absolutePointTurn =
      new AvailableCppCommandDeclaration((void *)AutoFunction::AbsolutePointTurn, "AbsolutePointTurn", {FLOAT});
  AvailableCppCommandDeclaration *align =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Align, "Align", {});

  AvailableCppCommandDeclaration *shift =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Wait, "Shift", {BOOLEAN});

  AvailableCppCommandDeclaration *wait =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Wait, "Wait", {FLOAT});

  std::vector<const AvailableCppCommandDeclaration *> commands = {
      driveStraight, driveStraightAtAngle, pointTurn, absolutePointTurn, align, shift, wait};
  
  std::ifstream ifs(auto_routine_file);  // Take in auto_routine_file
  
  try {
    compiler = new lemonscript::LemonScriptCompiler(ifs, commands, &state);

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
