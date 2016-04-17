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
  std::vector<std::vector<cv::Point>> contours = getAllContours(image);

  // index of best target
  unsigned int bestTarget = 0;
  // score of best target
  double bestScore = 0;
  for (unsigned int i = 0; i < contours.size(); i++) {
    // it isn't a shape if it has 2 points
    if (contours[i].size() >= 2) {
      cv::approxPolyDP(contours[i], contours[i], 2, true);
      double score = ScoreContour(contours[i], cv::Size(image.cols, image.rows));
      if(score > bestScore) {
        bestScore = score;
        bestTarget = i;
      }
    }
  }
  // only set points if points exist
  if (bestScore == 0) points_.clear();
  else points_ = contours[bestTarget];

  was_found = bestScore != 0;

  // update coordinates of best target
  if (points_.size() >= 2) {
    cv::Rect bounds = cv::boundingRect(points_);
    best_x = (bounds.br().x + bounds.tl().x) / 2;
    best_y = (bounds.br().y + bounds.tl().y) / 2;
  }

  score_=bestScore;
}

double ShapeDetector::ScoreContour(const std::vector<cv::Point>& contour, cv::Size image_size) const {
  // get distance from previous x and y
  cv::Rect bounding_box = cv::boundingRect(contour);
  double x = (bounding_box.br().x + bounding_box.tl().x) / 2.0;
  double y = (bounding_box.br().y + bounding_box.tl().y) / 2.0;

  double dx = x - best_x, dy = y - best_y;
  double distance_from_previous = std::sqrt(dx*dx + dy*dy) / bounding_box.width;

  double contour_width = static_cast<double>(bounding_box.width) / image_size.width;
  double contour_height = static_cast<double>(bounding_box.height) / image_size.height;

  double score_shape = kShapeWeight * ( - std::abs(contour.size() - 8.0));

  // Distance being 0 breaks calculations
  distance_from_previous = std::max(distance_from_previous, .01);

  double score_distance = was_found ? kDistanceWeight * (1 - distance_from_previous) : 0;
  double score_width = kWidthWeight * contour_width;

  return std::max(score_width + score_distance + score_shape, .01);
}

std::vector<std::vector<cv::Point>> ShapeDetector::getAllContours(cv::Mat m) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(m.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE,
                   cv::Point(0, 0));
  m.release();
  return contours;
}
