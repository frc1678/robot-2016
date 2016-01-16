#include "WPILib.h"

// include guards
#ifndef CITRUSBUTTON_H
#define CITRUSBUTTON_H

// Designed to get input from joysticks & manipulators.
class CitrusButton {
 protected:
  bool output;
  bool oldInput;


 public:
  CitrusButton(Joystick* tstick, int tbutton);

  ~CitrusButton();

  // All of these have clones with generic input. If we ever end up
  // needing a button that's not from a joystick.

  // Call at the end of every loop (once per loop)!
  void Update(bool input);

  void Update();

  bool ButtonClicked(bool input);

  bool ButtonClicked();

  bool ButtonReleased(bool input);

  bool ButtonReleased();

  bool ButtonPressed(bool input);

  bool ButtonPressed();

  bool ButtonState();

  // Reset to factory settings!
  void Reset();

  // Optional.
  Joystick* stick;
  int button;
};

class CitrusAxis : public CitrusButton {
  public:
    void Update();
    void Update(bool input);
};

// Use the following like: input = TurnOn(myButton); input = Toggle(myButton,
// input);
bool TurnOn(CitrusButton* button);

bool TurnOff(CitrusButton* button);

bool Toggle(CitrusButton* button, bool input);

#endif  // CITRUSBUTTON_H
