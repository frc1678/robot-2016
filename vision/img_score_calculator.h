#ifndef _IMG_SCORE_CALCULATOR_H_
#define _IMG_SCORE_CALCULATOR_H_

#include "opencv2/core.hpp"

class ImgScoreRule {
public:
  virtual void setWeights(cv::Mat reference)=0;
  cv::Mat getMask() { return mask; }
protected:
  cv::Mat mask;
};

class ImgScoreCalculator {
public:
  ImgScoreCalculator(ImgScoreRule* rule, cv::Mat reference);
  double score(cv::Mat image);
protected:
  ImgScoreRule* scoreRule;
};

#endif
