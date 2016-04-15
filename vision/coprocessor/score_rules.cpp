#include "score_rules.h"

void ScoreSelectAll::setWeights(cv::Mat reference) {
  mask.data=nullptr;
}

void ScoreIgnoreBlack::setWeights(cv::Mat reference) {
  mask=reference;
}
