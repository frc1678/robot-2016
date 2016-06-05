#ifndef LIGHTS_H
#define LIGHTS_H

#include "frc1678/frc1678.h"
#include <memory>

enum class ColorLight { RED = 0, YELLOW, GREEN, TEAL, BLUE, WHITE, PINK, OFF };

struct LightData {
  bool arm_done;
  bool tuck_def_pos;
  bool intake_pos;
  bool shoot_pos;
  bool disabled;
  bool vision_sees;
  bool vision_aligned;
  bool vision_connected;
  bool shooter_ready;
  bool gyro_calibrated;
  bool ball_intaked;
  std::string auto_selection;
};

class LightController {
 public:
  LightController();
  void UpdateLights();

  LightData light_data_;

 private:
  std::unique_ptr<DigitalOutput> l_pow_, l_red_, l_green_, l_blue_;

  ColorLight lights_;

  bool dorumble_;

  Time time = 0 * s;

  void ColorLights(ColorLight color);
  void SetLightColor(int r, int g, int b);
  ColorLight FlashLights(ColorLight color_one, ColorLight color_two,
                         bool off_between = false);
  bool ShouldRumble();
};

#endif
