#ifndef _SCORE_RULES_H_
#define _SCORE_RULES_H_

#include "img_score_calculator.h"

class ScoreSelectAll : public ImgScoreRule {
public:
  virtual void setWeights(cv::Mat reference);
}

class ScoreIgnoreBlack : public ImgScoreRule {
public:
  virtual void setWeights(cv::Mat reference);
}

#endif
