#ifndef _DATA_SENDER_H_
#define _DATA_SENDER_H_

#include "opencv2/core.hpp"
#include "object_detector.h"

namespace vision {
void startSending();
void updateData(cv::Mat image, TrackerResults position);
}

#endif