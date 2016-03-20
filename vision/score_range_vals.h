#ifndef _SCORE_RANGE_VALS_H_
#define _SCORE_RANGE_VALS_H_

#include "maximization.h"
#include "calibration.h"
#include "img_score_calculator.h"

class ScoreRangeVals : public ThingToMaximize {
protected:
  CalibrationInput images;
  int color;
  cv::Mat diff;
  ImgScoreCalculator scorePos, scoreNeg;
  int getDiffThresh();
  double weightForIteration(int iteration) const;
public:
  ScoreRangeVals(int colorspace, CalibrationInput dataSet,
      ImgScoreRule *negRule, ImgScoreRule *posRule);
  int getNumInputs() const;
  int getNumIterations() const;
  double getScore(std::vector<double> inputs, int iteration) const;
  std::vector<double> getStartEstimate() const;
  double getStepSize(int iteration) const;
};

#endif
