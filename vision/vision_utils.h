#ifndef _VISION_UTILS_H_
#define _VISION_UTILS_H_

#include "opencv2/core.hpp"
#include "unitscpp/unitscpp.h"
#include <string>

namespace vision {
Angle AngleBetweenPoints(cv::Point pi, cv::Point center, cv::Point p2);
// TODO(Lucas): add the other utils files
}

#endif
