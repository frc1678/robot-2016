#ifndef _IN_RANGE_INSTRUCTIONS_H_
#ifndef _IN_RANGE_INSTRUCTIONS_H_

#include <string>
#include "opencv2/core.hpp"
#include <string>

class InRangeInstructions {
 public:
  InRangeInstructions(std::String filename);
  InRangeInstructions(cv::Scalar low, cv::Scalar high, int colorspace);
  void Thresh(cv::Mat image);
  void WriteInstructions(std::string filename);

 private:
  int colorspace_;
  cv::Scalar low_, high_;
}
#endif
