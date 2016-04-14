#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include "in_range_instructions.h"
#include "opencv2/core.hpp"
#include <vector>

class CalibrationInput {
public:
  CalibrationInput(cv::Mat light_on, cv::Mat light_off, std::vector<cv::Mat> negatives);
  void applyThresh(InRangeInstructions thresh);
  cv::Mat on;
  cv::Mat off;
  std::vector<cv::Mat> extraNegatives;
};

namespace vision {
InRangeInstructions calibrate(CalibrationInput images);
}

#endif
