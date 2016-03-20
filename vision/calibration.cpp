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
  std::vector<double> scores;
  int bestColorspace=0;
  double bestScore=-100000;
  for(int i=0; i<colorspaces.size(); i++) {
    ScoreRangeVals scorer(colorspaces[i], images, scoreIgnoreBlack(), scoreSelectAll());
    Maximization bestRangeFinder(&scorer);;
    scores.push_back(bestRangeFinder.maximize());
    std::vector<double> maximizedValues=bestRangeFinder.getVals();
    ranges.push_back(InRangeInstructions(
        cv::Scalar(maximizedValues[0], maximizedValues[1], maximizedValues[2]),
        cv::Scalar(maximizedValues[3], maximizedValues[4], maximizedValues[5]),
        colorspaces[i]));
  }
  //TODO(Lucas) add selection
}
