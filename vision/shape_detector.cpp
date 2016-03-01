#include "shape_detector.h"
#include <cmath>

ShapeDetector::ShapeDetector(std::vector<Angle> angles) {
  angles_ = angles;
  points_ = std::vector<cv::Point>();
  best_x = 0;
  best_y = 0;
  score_ = 0;
}

void ShapeDetector::setData(cv::Mat image) {
  std::vector<std::vector<cv::Point>> contours = getAllContours(image);

  unsigned int closestToPrevious = 0;
  double bestDistance = 10000;
  for (unsigned int i = 0; i < contours.size(); i++) {
    if (points_.size() >= 2) {
      cv::Rect bounds = cv::boundingRect(contours[i]);
      double x = (bounds.br().x + bounds.tl().x) / 2;
      double y = (bounds.br().y + bounds.tl().y) / 2;
      if ((x - best_x) * (x - best_x) + (y - best_y) * (y - best_y) <
          bestDistance) {
        closestToPrevious = i;
        bestDistance =
            (x - best_x) * (x - best_x) + (y - best_y) * (y - best_y);
      }
    }
    double bestTarget = 0;
    for (unsigned int i = 0; i < contours.size(); i++) {
      std::vector<cv::Point> currentPoints = convertToPolygon(contours[i]);
      if (getTargetCertainty(currentPoints, i == closestToPrevious) >=
          bestTarget) {
        points_ = currentPoints;
        bestTarget = getTargetCertainty(points_, i == closestToPrevious);
      }
    }
    if (points_.size() >= 2) {
      cv::Rect bounds = cv::boundingRect(points_);
      best_x = (bounds.br().x + bounds.tl().x) / 2;
      best_y = (bounds.br().y + bounds.tl().y) / 2;
    }
  }
}

std::vector<std::vector<cv::Point>> ShapeDetector::getAllContours(cv::Mat m) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(m, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
                   cv::Point(0, 0));
  return contours;
}

double ShapeDetector::getTargetCertainty(std::vector<cv::Point> points,
                                         bool isClosestToPrevious) {
  // TODO(Lucas): add actual code
  return std::log(cv::boundingRect(points).width);
}

double getAngleDiff(std::vector<cv::Point> points, int offset) {
  // TODO(Lucas): add actual code
  return 0;
}
