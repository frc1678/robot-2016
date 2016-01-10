#include "frc1678/CitrusButton.h"

CitrusButton::CitrusButton(Joystick* tstick, int tbutton) {
  output = false;
  oldInput = false;
  stick = tstick;
  button = tbutton;
}

CitrusButton::~CitrusButton() {}

// All of these have clones with generic input. If we ever end up needing a
// button that's not from a joystick.

// Call at the end of every loop (once per loop)!
void CitrusButton::Update(bool input) { oldInput = input; }

void CitrusButton::Update() { Update(stick->GetRawButton(button)); }

bool CitrusButton::ButtonClicked(bool input) {
  // Return true if the button state changes from false to true. (not
  // clicked to clicked)
  bool returnMe = false;
  if (input != oldInput && input == true) {
    returnMe = true;
  }
  return returnMe;
}

bool CitrusButton::ButtonClicked() {
  // Return true if the button state changes from false to true. (not
  // clicked to clicked)
  if (button > 0) {
    return ButtonClicked(stick->GetRawButton(button));
  }
  return false;
}

bool CitrusButton::ButtonReleased(bool input)
// Return true if the button state changes from true to false. (clicked to not
// clicked)
{
  bool returnMe = false;
  if (input != oldInput && input == false) {
    returnMe = true;
  }
  return returnMe;
}

bool CitrusButton::ButtonReleased() {
  // Return true if the button state changes from true to false. (clicked
  // to not clicked)
  if (button > 0) {
    return ButtonReleased(stick->GetRawButton(button));
  }
  return false;
}

bool CitrusButton::ButtonPressed(
    bool input)  // only purpose is below. Why done this way?
{
  return input;
}

bool CitrusButton::ButtonPressed()  // returns the state of the button
{
  if (button > 0) {
    return ButtonPressed(stick->GetRawButton(button));
  }
  return false;
}

// Reset to factory settings!
void CitrusButton::Reset() {
  output = false;
  oldInput = false;
}

bool CitrusButton::ButtonState() { return output; }

// Use the following like: input = TurnOn(myButton); input = Toggle(myButton,
// input);
bool TurnOn(CitrusButton* button)  // used to undo below
{
  if (button->ButtonClicked()) {
    return true;
  }
  return false;
}

bool TurnOff(CitrusButton* button)  // if the button is clicked (see above) then
                                    // return false
{
  if (button->ButtonClicked()) {
    return false;
  }
  return true;
}

bool Toggle(CitrusButton* button, bool input)
// Toggle the results of clicking the button (see above). Used for clicked only.
// not pressed.
{
  if (button->ButtonClicked()) {
    return !input;
  }
  return input;
}
