#ifndef _DATA_SENDER_H_
#define _DATA_SENDER_H_

#include "opencv2/core.hpp"
#include "object_detector.h"

namespace vision {
void updateData(TrackerResults position);
void startSending();
}

#endif
