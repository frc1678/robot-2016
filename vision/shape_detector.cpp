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

  // find what index is closest to previous best shape, to reduce noise
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
  }

  // get index of best target
  double bestTarget = 0;
  for (unsigned int i = 0; i < contours.size(); i++) {
    std::vector<cv::Point> currentPoints = convertToPolygon(contours[i], image);
    if (getTargetCertainty(currentPoints, i == closestToPrevious) >=
        bestTarget) {
      points_ = currentPoints;
      bestTarget = getTargetCertainty(points_, i == closestToPrevious);
    }
  }

  // update coordinates of best target
  if (points_.size() >= 2) {
    cv::Rect bounds = cv::boundingRect(points_);
    best_x = (bounds.br().x + bounds.tl().x) / 2;
    best_y = (bounds.br().y + bounds.tl().y) / 2;
  }

  score_=bestTarget;
}

std::vector<std::vector<cv::Point>> ShapeDetector::getAllContours(cv::Mat m) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(m.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE,
                   cv::Point(0, 0));
  return contours;
}

std::vector<cv::Point> ShapeDetector::convertToPolygon(std::vector<cv::Point> points, const cv::Mat &image) {
  // if it is too small don't do any processing
  //if(cv::boundingRect(points).area() < image.rows * image.cols / 200) return points;
  std::vector<cv::Point> retval;

  // attempt to get the contour as a polygon with angles_.size() sides
  double epsilon=0;
  do {
    cv::approxPolyDP(points, retval, epsilon, true);
    epsilon+=0.5;
  } while(retval.size()>angles_.size());

  return retval;
}
double ShapeDetector::getTargetCertainty(std::vector<cv::Point> points,
                                         bool isClosestToPrevious) {
  if(points.size()==0) return -1;
  if(points.size()!=angles_.size()) {
    return std::log(cv::boundingRect(points).area());
  }
  double bestScore=0;

  // the angles in points may not be in the same order as the inputed ones,
  // try lining them up all ways
  for(unsigned int i=0; i<angles_.size(); i++) {
    double score=getAngleDiff(points, i);
    if(score>bestScore) bestScore=score;
  }
  if(isClosestToPrevious) bestScore+=8;
  return bestScore;
}

double ShapeDetector::getAngleDiff(std::vector<cv::Point> points, int offset) {
  double retval=0;

  // subtract differences in angle
  for(unsigned int i=0; i<points.size(); i++) {
    retval-=muan::abs(angles_[(offset+i)%points.size()]-vision::AngleBetweenPoints(
          points[i%points.size()],
          points[(i+1)%points.size()],
          points[(i+2)%points.size()])).to(rad);
  }

  // add some so it isn't negative
  retval+=1.571*points.size();

  // give it a bit higher score based on size
  retval*=std::log(cv::boundingRect(points).width+1);
  return retval;
}
