#include "frc1678/CitrusButton.h"

CitrusButton::CitrusButton(Joystick* tstick, int tbutton) {
  current = old = false;
  stick = tstick;
  button = tbutton;
}

CitrusButton::~CitrusButton() {}

// All of these have clones with generic input. If we ever end up needing a
// button that's not from a joystick.

// Call at the end of every loop (once per loop)!
void CitrusButton::Update(bool input) {
  old = current;
  current = input;
}

void CitrusButton::Update() { Update(stick->GetRawButton(button)); }

CitrusAxis::CitrusAxis(Joystick* stick, int button)
    : CitrusButton(stick, button) {}

void CitrusAxis::Update() {
  bool bclicked;
  bclicked = false;
  if (abs(stick->GetRawAxis(button)) >= .7) {
    bclicked = true;
  } else {
    bclicked = false;
  }
  CitrusButton::Update(bclicked);
}

bool CitrusButton::ButtonClicked() {
  // Return true if the button state changes from false to true. (not
  // clicked to clicked)
  return (current && !old);
}

bool CitrusButton::ButtonReleased() { return (!current && old); }

bool CitrusButton::ButtonPressed() { return current; }

// Reset to factory settings!
void CitrusButton::Reset() { current = old = false; }

bool CitrusButton::ButtonState() { return current; }

CitrusPOV::CitrusPOV(Joystick* joy, int pov, POVPosition pos)
    : CitrusButton(joy, pov) {
  position = pos;
}

void CitrusPOV::Update() {
  CitrusButton::Update(stick->GetPOV(button) == static_cast<int>(position));
}
