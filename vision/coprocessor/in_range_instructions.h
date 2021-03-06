#ifndef _IN_RANGE_INSTRUCTIONS_H_
#define _IN_RANGE_INSTRUCTIONS_H_

#include <string>
#include "opencv2/opencv.hpp"
#include <string>

class InRangeInstructions {
 public:
  InRangeInstructions(std::string filename);
  InRangeInstructions(cv::Scalar low, cv::Scalar high, int colorspace);
  void Thresh(cv::Mat& image);
  void WriteInstructions(std::string filename);
  void Set(cv::Scalar low, cv::Scalar high);

 private:
  int colorspace_;
  cv::Scalar low_, high_;
};
#endif
