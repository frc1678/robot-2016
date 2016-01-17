#include "auto_routines.h"
#include "auto_functions.h"
#include "lemonscript/lemonscript/AvailableCppCommandDeclaration.h"
#include <vector>
#include <iostream>

LemonScriptRunner::LemonScriptRunner(const std::string &auto_routine_file, CitrusRobot* robot) {
  state.userData = robot; // Whatever goes in userData comes out later.

  AvailableCppCommandDeclaration *driveStraight2 = new AvailableCppCommandDeclaration((void *)AutoFunction::DriveStraight2, "DriveStraight", {FLOAT, FLOAT});

  std::vector<const AvailableCppCommandDeclaration *> commands = {driveStraight2};

  std::ifstream ifs(auto_routine_file); // Take in auto_routine_file

  try {
    compiler = new LemonScriptCompiler(ifs, commands, &state);

  } catch (std::string error) {
    std::cerr << error << std::endl;

  }
}

LemonScriptRunner::~LemonScriptRunner() {
  delete compiler;

}

void LemonScriptRunner::Update() {
  compiler->PeriodicUpdate();

}
