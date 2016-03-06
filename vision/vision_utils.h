#ifndef _VISION_UTILS_H_
#define _VISION_UTILS_H_

#include "opencv2/core.hpp"
#include "unitscpp/unitscpp.h"
#include <string>
#include <vector>
#include "opencv2/imgproc.hpp"

namespace vision {
Angle AngleBetweenPoints(cv::Point pi, cv::Point center, cv::Point p2);
int getColorNumber(std::string colorName);
std::string getColorName(int colorNumber);

const std::vector<int> colorspaces={
  CV_BGR2HLS, CV_BGR2HSV, CV_BGR2Lab, CV_BGR2Luv,
  CV_BGR2RGB, CV_BGR2YCrCb, CV_BGR2YUV
};

const std::vector<std::string> colorNames={
  "HLS", "HVS", "Lab", "Luv", "RGB", "YCrCb", "YUV"
};
}

#endif
