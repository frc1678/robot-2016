#ifndef VISION_OBJECT_DETECTOR_H_
#define VISION_OBJECT_DETECTOR_H_

#include "opencv2/core.hpp"
#include "unitscpp/unitscpp.h"

struct TrackerResults {
  Angle angle;
  bool is_found;
};

class ObjectTracker {
 public:
  ObjectTracker();
  ~ObjectTracker();
  TrackerResults Update(cv::Mat& image);

 private:
  InRangeIntructions range;
};

#endif /* VISION_OBJECT_DETECTOR_H_ */
