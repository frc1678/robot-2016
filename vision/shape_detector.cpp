#include "shape_detector.h"
#include <cmath>
#include "muan/utils/math_utils.h"
#include "vision_utils.h"
#include <iostream>

ShapeDetector::ShapeDetector(std::vector<Angle> angles) {
  angles_ = angles;
  points_ = std::vector<cv::Point>();
  best_x = 0;
  best_y = 0;
  score_ = 0;
}

std::vector<cv::Point> ShapeDetector::getPoints() { return points_; }
double ShapeDetector::getScore() { return score_; }

void ShapeDetector::setData(cv::Mat image) {
  // constants for detection weight
  double distanceWeight = 1;
  double areaWeight = 0.1;
  double numSidesWeight = 0;

  std::vector<std::vector<cv::Point>> contours = getAllContours(image);

  // index of best target
  unsigned int bestTarget = 0;
  // score of best target
  double bestScore = 0;
  for (unsigned int i = 0; i < contours.size(); i++) {
    // it isn't a shape if it has 2 points
    if (contours[i].size() >= 2) {
      // get distance from previous x and y
      cv::Rect bounds = cv::boundingRect(contours[i]);
      double x = (bounds.br().x + bounds.tl().x) / 2;
      double y = (bounds.br().y + bounds.tl().y) / 2;
      double distanceFromPrevious =
            (x - best_x) * (x - best_x) + (y - best_y) * (y - best_y);
      // because it can't be 0
      if (distanceFromPrevious == 0) distanceFromPrevious = 0.01;
      // a weight of 1/distance, sqrt(area), and difference between number of sides
      double score = distanceWeight / distanceFromPrevious +
              areaWeight * std::sqrt(cv::boundingRect(contours[i]).area()) +
              numSidesWeight * (angles_.size() - contours[i].size() > 0 ?
                      angles_.size() - contours[i].size() :
                      contours[i].size() - angles_.size());
      if(score > bestScore) {
        bestScore = score;
        bestTarget = i;
      }
    }
  }
  // only set points if points exist
  if (bestScore == 0) points_.clear();
  else points_ = contours[bestTarget];

  // update coordinates of best target
  if (points_.size() >= 2) {
    cv::Rect bounds = cv::boundingRect(points_);
    best_x = (bounds.br().x + bounds.tl().x) / 2;
    best_y = (bounds.br().y + bounds.tl().y) / 2;
  }

  score_=bestScore;
}

std::vector<std::vector<cv::Point>> ShapeDetector::getAllContours(cv::Mat m) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(m.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE,
                   cv::Point(0, 0));
  m.release();
  return contours;
}
