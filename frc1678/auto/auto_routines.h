#ifndef AUTO_ROUTINES_H
#define AUTO_ROUTINES_H

#include <string>
#include "lemonscript/lemonscript/LemonScriptCompiler.h"
#include "lemonscript/lemonscript/LemonScriptState.h"
#include "lemonscript/lemonscript/AvailableCppCommandDeclaration.h"

class CitrusRobot;

// Class that will run LemonScript on the Robot
// Will set up function pointers for LemonScript
// Will allow someone to start a specific auto routine

class LemonScriptRunner {
 public:
  LemonScriptRunner(const std::string& auto_routine_file,
                    CitrusRobot* robot);
  ~LemonScriptRunner();
  void Update();

 private:
  lemonscript::LemonScriptCompiler* compiler;
  lemonscript::LemonScriptState state;
};

#endif
