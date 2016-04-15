#ifndef _DATA_SENDER_H_
#define _DATA_SENDER_H_

#include "opencv2/core.hpp"
#include "object_detector.h"

namespace vision {
// Set the results of image processing that will be sent over
void updateData(TrackerResults position);
// Run the sending of data. This is an infinite loop, so run in a seperate thread.
void startSending();
}

#endif
