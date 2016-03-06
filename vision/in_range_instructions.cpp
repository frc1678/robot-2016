#include "in_range_instructions.h"
#include "vision_utils.h"
#include "opencv2/imgproc.hpp"
#include <fstream>
#include <regex>
#include <sstream>

InRangeInstructions::InRangeInstructions(std::string filename) {
  std::ifstream file(filename);
  std::string str, buffer;
  while(file) {
    std::getline(file, buffer);
    str+=buffer;
  }
  str=std::regex_replace(str, std::regex("[\\D]"), " ");
  std::stringstream ss;
  ss<<str;
  ss>>colorspace_;
  int num1, num2, num3;
  ss>>num1>>num2>>num3;
  low_ = cv::Scalar(num1, num2, num3);
  ss>>num1>>num2>>num3;
  high_ = cv::Scalar(num1, num2, num3);
}

InRangeInstructions::InRangeInstructions(cv::Scalar low, cv::Scalar high,
                                         int colorspace) {
  low_ = low;
  high_ = high;
  colorspace_ = colorspace;
}

void InRangeInstructions::Thresh(cv::Mat& image) {
  cv::cvtColor(image, image, colorspace_);
  cv::inRange(image, low_, high_, image);
}

void InRangeInstructions::WriteInstructions(std::string filename) {
  std::ofstream file(filename);
  file<<"Auto-generated using Citrus Vision 2016, c++ version\nColorspace: " <<
    colorspace_ << " (" << vision::getColorName(colorspace_) << ")\nlow=(" <<
    low_.val[0] << ", " << low_.val[1] << ", " << low_.val[2] << ")\nhigh=(" <<
    high_.val[0] << ", " << high_.val[1] << ", " <<high_.val[2] << ")";
}
