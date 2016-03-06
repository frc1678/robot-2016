#include "calibration.h"
#include "vision_utils.h"

CalibrationInput::CalibrationInput(cv::Mat light_on, cv::Mat light_off,
    std::vector<cv::Mat> negatives) {
  on=light_on;
  off=light_off;
  extraNegatives=negatives;
}

void CalibrationInput::applyThresh(InRangeInstructions thresh) {
  thresh.thresh(on);
  thresh.thresh(off);
  for(int i=0; i<negatives.size(); i++) {
    thresh.thresh(negatives[i]);
  }
}

InRangeInstructions vision::calibrate(CalibrationInput images) {
  std::vector<InRangeInstructions> ranges;
  int bestColorspace=0;
  double bestScore=(~0);
  for(int i=0; i<colorspaces.size(); i++) {
    ScoreRangeVals* scorer=new ScoreRangeVals(colorspaces[i]);
