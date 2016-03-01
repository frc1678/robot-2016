#include "in_range_instructions.h"
#include "opencv2/imgproc.hpp"
InRangeInstructions::InRangeInstructions(std::string filename) {
  // TODO(Lucas) read from file
  low_ = cv::Scalar(0, 140, 140);
  high_ = cv::Scalar(125, 255, 255);
  colorspace_ = 4;
}

InRangeInstructions::InRangeInstructions(cv::Scalar low, cv::Scalar high,
                                         int colorspace) {
  low_ = low;
  high_ = high;
  colorspace_ = colorspace;
}

void InRangeInstructions::Thresh(cv::Mat image) {
  cv::cvtColor(image, image, colorspace_);
  cv::inRange(image, low_, high_, image);
}

void WriteInstructions(std::string filename) {
  // TODO(Lucas): write to file
}
