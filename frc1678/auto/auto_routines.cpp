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

  AvailableCppCommandDeclaration *driveStraight2 =
      new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraight2, "DriveStraight", {FLOAT, FLOAT});
  
  AvailableCppCommandDeclaration *pointTurn =
      new AvailableCppCommandDeclaration((void *)AutoFunction::PointTurn, "PointTurn", {FLOAT, FLOAT});

  AvailableCppCommandDeclaration *align =
      new AvailableCppCommandDeclaration((void *)AutoFunction::Align, "Align", {});

  std::vector<const AvailableCppCommandDeclaration *> commands = {
      driveStraight2, pointTurn, align};
  
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
