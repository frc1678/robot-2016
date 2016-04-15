#include "score_range_vals.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

ScoreRangeVals::ScoreRangeVals(int colorspace, CalibrationInput dataSet,
    ImgScoreRule *negRule, ImgScoreRule *posRule) {
  images=dataSet;
  color=colorspace;
  scorePos=ImgScoreCalculator(posRule);
  scoreNeg=ImgScoreCalculator(negRule);
  diff=cv::Mat(images.on.rows(), images.on.cols(), CV_8U);
  cv::absdiff(images.on, images.off, diff);
  cv::cvtColor(diff, diff, CV_BGR2GRAY);
  int diffThresh=getDiffThresh();
  std::cout<<"diff thresh: "<<diffThresh<<std::endl;
  cv::threshold(diff, diff, diffThresh, 255, CV_THRESH_BINARY);
  cv::erode(diff, diff, cv::getStructuringElement(CV_MORPH_RECT, cv::Size(7, 7), cv::Point(3, 3)));
  cv::namedWindow("calibration", CV_WINDOW_AUTOSIZE);
}

int ScoreRangeVals::getDiffThresh() {
  //TODO(Lucas) add actual code
  return 40;
}

double ScoreRangeVals::weightForIteration(int iteration) const {
  switch(iteration) {
  case 1:
    return 1;
  case 2:
    return 5;
  case 3:
    return 10;
  case 4:
    return 20;
  case 5:
    return 50;
  default:
    return 0;
  }
}

int ScoreRangeVals::getNumInputs() const {
  return 6;
}

int ScoreRangeVals::getNumIterations() const {
  return 5;
}

double ScoreRangeVals::getStepSize(int iteration) const {
  switch(iteration) {
  case 1:
    return 20;
  case 2:
    return 10;
  case 3:
    return 5;
  case 4:
    return 5;
  case 5:
    return 5;
  default:
    return 0;
  }
}

std::vector<double> ScoreRangeVals::getStartEstimate() const {
  return std::vector<double>({0, 0, 0, 255, 255, 255});
}

double ScoreRangeVals::getScore(std::vector<double> inputs, int iteration) const {
  InRangeInstructions thresh(cv::Scalar(inputs[0], inputs[1], inputs[2]), cv::Scalar(inputs[3], inputs[4], inputs[5]), color);
  CalibrationInput imageCopy=images;
  imageCopy.applyThresh(thresh);
  cv::imshow("calibration", imageCopy.on);
  double retval=scorePos.score(imageCopy.on)-weightForIteration(iteration)*scoreNeg.score(imageCopy.off);
  for(int i=0; i<imageCopy.extraNegatives.size(); i++) {
    retval-=weightForIteration(iteration)*scoreNeg.score(imageCopy.extraNegatives[i]);
  }
  return retval;
}
