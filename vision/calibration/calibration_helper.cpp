#include <string>
#include <vector>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "vision/coprocessor/object_detector.h"

void Usage() {
  std::cout << "Usage: calibration_helper [tune|capture]\n"
            << "capture\n"
            << "  --filename [file]\n"
            << "  --camera [camera id]\n"
            << "tune\n"
            << "  --filename [file]\n"
            << "  --output [param filename]" << std::endl;
}

void Capture(const std::string& filename, int camera_index = -1) {
  cv::VideoCapture camera(camera_index);
  if (!camera.isOpened()) {
    std::cout << "Could not open camera " << camera_index << std::endl;
    return;
  }

  camera.set(CV_CAP_PROP_BUFFERSIZE, 0);
  camera.set(CV_CAP_PROP_BRIGHTNESS, 0);  // minimum
  camera.set(CV_CAP_PROP_CONTRAST, 1);    // maximun
  camera.set(CV_CAP_PROP_SATURATION, 1);  // maximun
  std::string command = "v4l2-ctl -d /dev/video" +
                        std::to_string(camera_index) +
                        " -c "
                        "exposure_auto=1,exposure_absolute=5,white_balance_"
                        "temperature_auto=0,"
                        "white_balance_temperature=8000";
  system(command.c_str());  // not best way, but it keeps the settings together

  int ex = CV_FOURCC('P', 'I', 'M', '1');
  cv::Size video_size = cv::Size(camera.get(cv::CAP_PROP_FRAME_WIDTH),
                                 camera.get(cv::CAP_PROP_FRAME_HEIGHT));
  cv::VideoWriter writer;
  if (!writer.open(filename, ex, 30, video_size, true)) {
    std::cout << "Could not open file " << filename << " for writing"
              << std::endl;
    return;
  }

  cv::Mat frame;
  cv::namedWindow("Capture");

  while (cv::waitKey(10) == -1) {
    camera >> frame;
    writer << frame;
    cv::imshow("Capture", frame);
  }
  camera.release();
  writer.release();
}

void Capture(std::vector<std::string> args) {
  int camera_index = -1;
  std::string filename = "";

  for (int i = 0; i < args.size() - 1; i++) {
    if (args[i] == "--filename") {
      filename = args[++i];
    } else if (args[i] == "--camera") {
      camera_index = std::stoi(args[++i]);
    }
  }

  if (filename == "") {
    std::cout << "No filename specified" << std::endl;
    Usage();
    return;
  } else {
    if (camera_index == -1) {
      std::cout << "No camera specified, using default" << std::endl;
    }

    Capture(filename, camera_index);
  }
}

void Tune(const std::string& video_filename,
          const std::string& output_filename) {
  cv::VideoCapture video(video_filename);
  if (!video.isOpened()) {
    std::cout << "Could not open video: " << video_filename << std::endl;
    return;
  }

  cv::namedWindow("Parameters");
  int h_min = 0, s_min = 0, v_min = 0;
  int h_max = 255, s_max = 255, v_max = 255;
  cv::createTrackbar("H min", "Parameters", &h_min, 255);
  cv::createTrackbar("S min", "Parameters", &s_min, 255);
  cv::createTrackbar("V min", "Parameters", &v_min, 255);
  cv::createTrackbar("H max", "Parameters", &h_max, 255);
  cv::createTrackbar("S max", "Parameters", &s_max, 255);
  cv::createTrackbar("V max", "Parameters", &v_max, 255);
  InRangeInstructions range(cv::Scalar(h_min, s_min, v_min),
                            cv::Scalar(h_max, s_max, v_max), 40);

  cv::namedWindow("Input");
  cv::namedWindow("Output");
  cv::Mat frame;
  while (cv::waitKey(10) == -1) {
    range.Set(cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max));
    video >> frame;
    if (frame.size[0] <= 0) {
      video.set(cv::CAP_PROP_POS_AVI_RATIO, 0);
      continue;
    }
    cv::imshow("Input", frame);
    range.Thresh(frame);
    cv::imshow("Output", frame);
  }
  range.WriteInstructions(output_filename);
}
void Tune(std::vector<std::string> args) {
  std::string filename = "", output = "";

  for (int i = 0; i < static_cast<int>(args.size()) - 1; i++) {
    if (args[i] == "--filename") {
      filename = args[++i];
    } else if (args[i] == "--output") {
      output = args[++i];
    }
  }

  if (filename == "") {
    std::cout << "No filename specified" << std::endl;
    Usage();
    return;
  } else {
    if (output == "") {
      std::cout << "No params file specified, using default PARAMS.txt"
                << std::endl;
    }

    Tune(filename, output);
  }
}

int main(int argc, const char** args) {
  if (argc >= 2) {
    std::string command(args[1]);

    std::vector<std::string> argv;
    for (int i = 2; i < argc; i++) {
      argv.push_back(std::string(args[i]));
    }

    if (command == "capture") {
      Capture(argv);
    } else if (command == "tune") {
      Tune(argv);
    } else {
      Usage();
    }
  } else {
    Usage();
  }
}
