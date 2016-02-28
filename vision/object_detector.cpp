#include "object_detector.h"

ObjectTracker::ObjectTracker() { range = InRangeIntructions("PARAMS.txt"); }

ObjectTracker::~ObjectTracker() {}

TrackerResults ObjectTracker::Update(cv::Mat& image) { range.Thresh(image); }
