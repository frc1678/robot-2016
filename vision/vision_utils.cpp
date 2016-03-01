#include "vision_utils.h"
#include "opencv2/highgui.hpp"

cv::Mat vision::getImage() {
  // TODO(Lucas): actally read from URL
  return cv::imread("light_on.jpg");
}
