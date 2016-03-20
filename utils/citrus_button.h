#include "WPILib.h"

// include guards
#ifndef CITRUSBUTTON_H
#define CITRUSBUTTON_H

// Designed to get input from joysticks & manipulators.
class CitrusButton {
 protected:
  bool current, old;

  // Optional.
  Joystick* stick;
  int button;

 public:
  CitrusButton(Joystick* tstick, int tbutton);

  ~CitrusButton();

  // All of these have clones with generic input. If we ever end up
  // needing a button that's not from a joystick.

  // Call at the end of every loop (once per loop)!
  void Update();
  void Update(bool input);

  bool ButtonClicked();

  bool ButtonReleased();

  bool ButtonPressed();

  bool ButtonState();

  // Reset to factory settings!
  void Reset();
};

class CitrusAxis : public CitrusButton {
 public:
  CitrusAxis(Joystick* stick, int button);
  void Update();
};

enum class POVPosition {
  NORTH = 0,
  NORTHEAST = 45,
  EAST = 90,
  SOUTHEAST = 135,
  SOUTH = 180,
  SOUTHWEST = 225,
  WEST = 270,
  NORTHWEST = 315
};

class CitrusPOV : public CitrusButton {
 public:
  CitrusPOV(Joystick* joy, int pov, POVPosition pos);
  void Update();

 protected:
  POVPosition position;
};

#endif  // CITRUSBUTTON_H
