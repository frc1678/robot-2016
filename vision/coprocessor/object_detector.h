#ifndef VISION_OBJECT_DETECTOR_H_
#define VISION_OBJECT_DETECTOR_H_

#include "opencv2/core.hpp"
#include "unitscpp/unitscpp.h"
#include "in_range_instructions.h"
#include "shape_detector.h"

struct TrackerResults {
  Angle angle;
  Time time_captured;
  bool is_found;
};

class ObjectTracker {
 public:
  ObjectTracker();
  ~ObjectTracker();
  TrackerResults Update(cv::Mat& image);

  void SetThresholds(cv::Scalar low, cv::Scalar high);

 private:
  InRangeInstructions range;
  ShapeDetector detector;
  const static Angle FOV;
  const static double minScore;
};

#endif /* VISION_OBJECT_DETECTOR_H_ */
