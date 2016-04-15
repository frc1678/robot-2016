#ifndef _SHAPE_DETECTOR_H_
#define _SHAPE_DETECTOR_H_

#include <vector>
#include "unitscpp/unitscpp.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

class ShapeDetector {
 public:
  ShapeDetector(std::vector<Angle> angles);
  // run detection on a black and white image
  void setData(cv::Mat image);
  // get the detected target
  std::vector<cv::Point> getPoints();
  // get the score of the target
  double getScore();

 private:
  std::vector<std::vector<cv::Point>> getAllContours(cv::Mat m);

  std::vector<Angle> angles_;
  std::vector<cv::Point> points_;
  double score_;
  double best_x, best_y;
};
#endif
