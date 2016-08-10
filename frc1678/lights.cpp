#include "frc1678/lights.h"
#include "muan/utils/timing_utils.h"
#include <WPILib.h>

LightController::LightController() {
  l_red_ = std::make_unique<DigitalOutput>(7);
  l_green_ = std::make_unique<DigitalOutput>(8);
  l_blue_ = std::make_unique<DigitalOutput>(9);
}

void LightController::UpdateLights() {
  ColorLight light_color;

  // for enabled
  if (light_data_.arm_done && !light_data_.tuck_def_pos &&
      !light_data_.vision_sees) {  // if arm is at position, not seeing vision
    light_color = ColorLight::RED;
  } else if (light_data_.vision_sees && !light_data_.tuck_def_pos &&
             light_data_.arm_done) {  // if vision sees target + arm is
                                      // done, yellow!
    light_color = ColorLight::YELLOW;
  }

  // Aligned and ready to shoot
  if (light_data_.vision_aligned && light_data_.shoot_pos &&
      !light_data_.tuck_def_pos && light_data_.arm_done &&
      light_data_.shooter_ready) {
    light_color = ColorLight::GREEN;
  }

  if (!light_data_.arm_done) {
    light_color = ColorLight::RED;
  }

  if (light_data_.arm_done && light_data_.tuck_def_pos) {
    light_color = ColorLight::GREEN;
  }

  if (!light_data_.ball_intaked && light_data_.intake_pos) {
    light_color = ColorLight::BLUE;
    time = 0 * s;
  } else if (light_data_.ball_intaked && light_data_.intake_pos) {
    light_color = ColorLight::GREEN;
    time += 0.02 * s;
    if (time < 1 * s && !light_data_.disabled) {
      dorumble_ = true;
    } else {
      dorumble_ = false;
    }
  } else {
    time = 0 * s;
    dorumble_ = false;
  }

  if (light_data_.disabled) {
    dorumble_ = false;

    if (!light_data_.gyro_calibrated) {
      light_color = ColorLight::BLUE;
    } else {
      SmartDashboard::PutString("auto", light_data_.auto_selection);
      if (light_data_.auto_selection == "one_ball.auto") {
        light_color = ColorLight::GREEN;
      } else if (light_data_.auto_selection == "two_ball.auto") {
        light_color = ColorLight::RED;
      } else if (light_data_.auto_selection == "class_d_left.auto") {
        light_color = FlashLights(ColorLight::YELLOW, ColorLight::GREEN);
      } else if (light_data_.auto_selection == "class_d_right.auto") {
        light_color = FlashLights(ColorLight::PINK, ColorLight::GREEN);
      } else if (light_data_.auto_selection == "class_a_left_right.auto") {
        light_color = FlashLights(ColorLight::YELLOW, ColorLight::BLUE);
      } else if (light_data_.auto_selection == "class_a_right_right.auto") {
        light_color = FlashLights(ColorLight::PINK, ColorLight::BLUE);
      } else if (light_data_.auto_selection == "class_a_left_left.auto") {
        light_color = FlashLights(ColorLight::YELLOW, ColorLight::RED);
      } else if (light_data_.auto_selection == "class_a_right_left.auto") {
        light_color = FlashLights(ColorLight::PINK, ColorLight::RED);
      } else {
        light_color = ColorLight::WHITE;
      }
    }
  }

  light_color =
      FlashLights(light_color, light_color, !light_data_.vision_connected);
  ColorLights(light_color);
}

void LightController::ColorLights(ColorLight value) {
  switch (value) {
    case ColorLight::RED:
      SetLightColor(1, 0, 0);
      break;
    case ColorLight::YELLOW:
      SetLightColor(1, 1, 0);
      break;
    case ColorLight::GREEN:
      SetLightColor(0, 1, 0);
      break;
    case ColorLight::BLUE:
      SetLightColor(0, 0, 1);
      break;
    case ColorLight::WHITE:
      SetLightColor(1, 1, 1);
      break;
    case ColorLight::PINK:
      SetLightColor(1, 0, 1);
      break;
    case ColorLight::TEAL:
      SetLightColor(0, 1, 1);
      break;
    case ColorLight::OFF:
      SetLightColor(0, 0, 0);
      break;
  }
}

void LightController::SetLightColor(int r, int g, int b) {
  l_red_->Set(r);
  l_green_->Set(g);
  l_blue_->Set(b);
}

ColorLight LightController::FlashLights(ColorLight color_one,
                                        ColorLight color_two,
                                        bool off_between) {
  auto color =
      (static_cast<int>(muan::now().to(s)) % 2) ? color_one : color_two;
  if (off_between && fmod(muan::now().to(s / 2), 0.5) < 0.25)
    color = ColorLight::OFF;
  return color;
}

bool LightController::ShouldRumble() { return dorumble_; }
