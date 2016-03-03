#include "object_detector.h"

ObjectTracker::ObjectTracker()
    : range(InRangeInstructions("PARAMS.txt")),
      detector(ShapeDetector(std::vector<Angle>(
          {20 * deg, -100 * deg, -100 * deg, 20 * deg, 80 * deg, 80 * deg}))) {}

ObjectTracker::~ObjectTracker() {}

TrackerResults ObjectTracker::Update(cv::Mat& image) {
  TrackerResults retval;
  range.Thresh(image);
  detector.setData(image);
  if (detector.getScore() > 0) {
    cv::cvtColor(image, image, CV_GRAY2BGR);
    cv::drawContours(
        image, std::vector<std::vector<cv::Point> >({detector.getPoints()}), 0,
        cv::Scalar(0, 255, 0), 3);
    cv::Rect boundRect = cv::boundingRect(detector.getPoints());
    retval.angle =
        FOV * ((boundRect.tl().x + boundRect.br().x) / (2 * image.cols) - 0.5);
    retval.is_found = true;
  } else {
    retval.angle = 0 * deg;
    retval.is_found = false;
  }
  return retval;
}

const Angle ObjectTracker::FOV = 55 * deg;
const double ObjectTracker::minScore = 20;
