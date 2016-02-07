#include "auto_routines.h"
#include "auto_functions.h"
#include "lemonscript/lemonscript/AvailableCppCommandDeclaration.h"
#include <vector>
#include <iostream>

LemonScriptRunner::LemonScriptRunner(const std::string &auto_routine_file,
                                     RobotSubsystems *subsystems) {
  state.userData = reinterpret_cast<int *>(
      subsystems);  // Whatever goes in userData comes out later.

  using lemonscript::AvailableCppCommandDeclaration;
  AvailableCppCommandDeclaration *driveStraight2 =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraight2, "DriveStraight", {FLOAT, FLOAT});
  
  AvailableCppCommandDeclaration *pointTurn =
      new AvailableCppCommandDeclaration((void *)AutoFunction::PointTurn, "PointTurn", {FLOAT, FLOAT});
  std::vector<const AvailableCppCommandDeclaration *> commands = {
      driveStraight2, pointTurn};
  
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
