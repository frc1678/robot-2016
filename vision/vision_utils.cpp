#include "vision_utils.h"
#include "opencv2/highgui.hpp"
#include <cmath>
#include <iostream>
#include "network_reader.h"

Angle vision::AngleBetweenPoints(cv::Point p1, cv::Point center, cv::Point p2) {
  return std::acos(
      //dot product
      ((p1.x-center.x)*(p2.x-center.x)+(p1.y-center.y)*(p2.y-center.y))/
      //length of the sides
      std::sqrt(((p1.x-center.x)*(p1.x-center.x)+(p1.y-center.y)*(p1.y-center.y))*
        ((p2.x-center.x)*(p2.x-center.x)+(p2.y-center.y)*(p2.y-center.y))))*
    //flip sign if cross product is negative
    (((p1.x-center.x)*(p2.y-center.y)-(p2.x-center.x)*(p1.y-center.y))<0?-1:1)*rad;
}
