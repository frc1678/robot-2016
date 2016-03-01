#ifndef _SHAPE_DETECTOR_H_
#define _SHAPE_DETECTOR_H_

#include <vector>
#include "unitscpp/unitscpp.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

class ShapeDetector {
 public:
  ShapeDetector(std::vector<Angle> angles);
  void setData(cv::Mat image);
  std::vector<cv::Point> getPoints();
  double getScore();

 private:
  std::vector<std::vector<cv::Point>> getAllContours(cv::Mat m);
  std::vector<cv::Point> convertToPolygon(std::vector<cv::Point> points);
  double getTargetCertainty(std::vector<cv::Point> points,
                            bool isClosestToPrevious);
  double getAngleDiff(std::vector<cv::Point> points, int offset);

  std::vector<Angle> angles_;
  std::vector<cv::Point> points_;
  double score_;
  double best_x, best_y;
};
#endif
