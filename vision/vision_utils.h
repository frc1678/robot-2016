#ifndef _VISION_UTILS_H_
#define _VISION_UTILS_H_

#include "opencv2/core.hpp"
#include <string>

namespace vision {
cv::Mat getImage(std::string URL);
// TODO(Lucas): add the other utils files
}

#endif
