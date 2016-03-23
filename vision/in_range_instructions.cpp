#include "in_range_instructions.h"
#include "vision_utils.h"
#include "opencv2/imgproc.hpp"
#include <fstream>
#include <regex>
#include <sstream>
#include <iostream>

InRangeInstructions::InRangeInstructions(std::string filename) {
  std::ifstream file(filename);
  std::string str, buffer;
  if(!file) {
    str = "4 0 100 100 100 255 255";
    std::cout<<filename<<" does not exist"<<std::endl;
  }
  else {
    while(file) {
      std::getline(file, buffer);
      str+=buffer;
    }
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
