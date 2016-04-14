#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "in_range_instructions.h"
#include "calibration.h"
#include "vision_utils.h"

int main() {
  InRangeInstructions range=vision::calibrate(cv::imread("light_on.jpg"), cv::imread("light_off.jpg"),
      std::vector<cv::Mat>({"negative.jpg"}));
  Mat image=cv::imread("light_on.jpg");
  range.thresh(image);
  range.writeInstructions("PARAMS.txt");
  cv::imwrite(image, "thresh.jpg");
  std::vector<std::string> files({"light_on.jpg", "light_off.jpg", "thresh.jpg", "negative.jpg", "PARAMS.txt"});
  vision::archive("files");
  cv::namedWindow("vision output");
  cv::imshow(image, "vision output");
  cv::waitKey(-1);
}
