#include "img_score_calculator.h"

ImgScoreCalculator::ImgScoreCalculator(ImgScoreRule* rule, cv::Mat reference) {
  scoreRule=rule;
  rule->setWeights(reference);
}

double ImgScoreCalculator::score(cv::Mat image) {
  if(scoreRule->getMask().data==nullptr) {
    return cv::mean(image).val[0];
  }
  return cv::mean(image, scoreRule->getMask()).val[0];
}
