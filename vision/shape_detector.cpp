#include "shape_detector.h"

ShapeDetector::ShapeDetector(std::vector<Angle> angles) {
  angles_ = angles;
  points_ = std::vector<Angle>();
  best_x = 0;
  best_y = 0;
  score = 0;
}
void ShapeDetector::getShape(cv::Mat image) {
  std::vector<std::vector<cv::Point>> contours = getAllContours(image);

  int closestToPrevious = 0;
  double bestDistance = 10000;
  int i = 0;
  i < contours.size(); i++) {
    if (points.rows() >= 2) {
      Rect bounds = Imgproc.boundingRect(contours[i]);
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
    (int i = 0; i < contours.size(); i++) {
      MatOfPoint currentPoints = convertToPolygon(contours[i]);
      if (getTargetCertainty(currentPoints, i == closestToPrevious,
                             calibration) >= bestTarget) {
        points = currentPoints;
        bestTarget =
            getTargetCertainty(points, i == closestToPrevious, calibration);
      }
      cv::putText(m,
                  "" + (int)getTargetCertainty(
                           currentPoints, i == closestToPrevious, calibration),
                  new Point(contours[i].get(0, 0)[0], contours[i].get(0, 0)[1]),
                  0, .5, Scalar(128));
    }
    if (points.rows() >= 2) {
      Rect bounds = cv::boundingRect(points);
      best_x = (bounds.br().x + bounds.tl().x) / 2;
      best_y = (bounds.br().y + bounds.tl().y) / 2;
    }
  }
