#include "in_range_instructions.h"
#include "vision_utils.h"
#include "opencv2/opencv.hpp"
#include <fstream>
#include <regex>
#include <sstream>
#include <iostream>

InRangeInstructions::InRangeInstructions(std::string filename) {
  std::ifstream file(filename);
  std::string str, buffer;

  // use default value, but output warning
  if (!file) {
    str = "4 0 100 100 100 255 255";
    std::cout << filename << " does not exist" << std::endl;
  }
  // read from file
  else {
    while (file) {
      std::getline(file, buffer);
      str += buffer;
    }
  }
  // ignore everything except numbers
  str = std::regex_replace(str, std::regex("[\\D]"), std::string(" "));

  std::stringstream ss;
  ss << str;
  ss >> colorspace_;
  int num1, num2, num3;
  ss >> num1 >> num2 >> num3;
  low_ = cv::Scalar(num1, num2, num3);
  ss >> num1 >> num2 >> num3;
  high_ = cv::Scalar(num1, num2, num3);
}

InRangeInstructions::InRangeInstructions(cv::Scalar low, cv::Scalar high,
                                         int colorspace) {
  low_ = low;
  high_ = high;
  colorspace_ = colorspace;
}

void InRangeInstructions::WriteInstructions(std::string filename) {
  std::ofstream file(filename);
  file << colorspace_ << " " << low_[0] << " " << low_[1] << " " << low_[2]
       << " " << high_[0] << " " << high_[1] << " " << high_[2] << '\n';
  file.close();
}

void InRangeInstructions::Thresh(cv::Mat& image) {
  cv::cvtColor(image, image, colorspace_);
  cv::inRange(image, low_, high_, image);
}

void InRangeInstructions::Set(cv::Scalar low, cv::Scalar high) {
  low_ = low;
  high_ = high;
}
