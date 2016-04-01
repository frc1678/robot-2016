#include "object_detector.h"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include "data_sender.h"
#include <thread>
#include "vision_utils.h"
#include <iostream>

int main() {
  cv::VideoCapture camera;

  // open last camera index
  int cameraIndex=0;
  for(int i=0; i<8; i++) {
    camera.open(i);
    if(camera.isOpened()) cameraIndex = i;
  }

  // exit if no camera can be opened
  camera.open(cameraIndex);
  if(!camera.isOpened()) return 0;

  camera.set(CV_CAP_PROP_BUFFERSIZE, 0);
  camera.set(CV_CAP_PROP_BRIGHTNESS, 0); // minimum
  camera.set(CV_CAP_PROP_CONTRAST, 1); // maximun
  camera.set(CV_CAP_PROP_SATURATION, 1); // maximun
  std::string command="v4l2-ctl -d /dev/video" + std::to_string(cameraIndex) +
          " -c exposure_auto=1 -c exposure_absolute=5"; // manual, minimum
  system(command.c_str()); // not best way, but it keeps the settings together

  ObjectTracker tracker = ObjectTracker();

  // for accurate lag measurements, run sender in another thread
  std::thread sender(vision::startSending);

  cv::namedWindow("detected", cv::WINDOW_AUTOSIZE);
  while (true) {
    cv::Mat image;

    // time is for lag measurement
    Time captureTime = muan::now();

    // exit if camera has lost connection
    // This does not work with normal OpenCV because of issue 5746.
    // Using https://github.com/amannababanana/opencv for fix.
    if(!camera.read(image)) return 0;

    cv::resize(image, image, cv::Size(image.cols / 3, image.rows / 3));
    TrackerResults position = tracker.Update(image);
    position.time_captured = captureTime;
    vision::updateData(position);
    if (image.data) {
      cv::imshow("detected", image);
      cv::waitKey(1);
    }
    image.release();
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  sender.join();
  return 0;
}
