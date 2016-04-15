#include "gtest/gtest.h"
#include "vision_utils.h"
#include "shape_detector.h"
#include "object_detector.h"

TEST(vision_utils, colorNames) {
  EXPECT_EQ(vision::colorNames.size(), vision::colorspaces.size());
  for(unsigned int i=0; i<vision::colorspaces.size(); i++) {
    EXPECT_EQ(vision::colorspaces[i], vision::getColorNumber(vision::getColorName(vision::colorspaces[i])));
    EXPECT_EQ(vision::colorNames[i], vision::getColorName(vision::getColorNumber(vision::colorNames[i])));
  }
}

TEST(vision_utils, pointsToAngle) {
  EXPECT_NEAR(0, vision::AngleBetweenPoints(cv::Point(1, 0), cv::Point(0, 0), cv::Point(1, 0)).to(deg), 0.01);
  EXPECT_NEAR(90, vision::AngleBetweenPoints(cv::Point(1, 0), cv::Point(0, 0), cv::Point(0, 1)).to(deg), 0.01);
  EXPECT_NEAR(-90, vision::AngleBetweenPoints(cv::Point(0, 1), cv::Point(0, 0), cv::Point(1, 0)).to(deg), 0.01);
  EXPECT_NEAR(45, vision::AngleBetweenPoints(cv::Point(1, 0), cv::Point(0, 0), cv::Point(1, 1)).to(deg), 0.01);
  EXPECT_NEAR(90, vision::AngleBetweenPoints(cv::Point(1, 1), cv::Point(0, 0), cv::Point(-1, 1)).to(deg), 0.01);
  EXPECT_NEAR(90, vision::AngleBetweenPoints(cv::Point(3, 1), cv::Point(2, 1), cv::Point(2, 2)).to(deg), 0.01);
  EXPECT_NEAR(180, vision::AngleBetweenPoints(cv::Point(1, 0), cv::Point(0, 0), cv::Point(-1, 0)).to(deg), 0.01);
}

TEST(ShapeDetector, zerosOnEmpty) {
  ShapeDetector detector({ 90 * deg, 90 * deg, 90 * deg, 90 * deg});
  EXPECT_EQ(0, detector.getScore());
  EXPECT_EQ(0, detector.getPoints().size());
}

TEST(ShapeDetector, zerosOnBlankMat) {
  ShapeDetector detector({ 90 * deg, 90 * deg, 90 * deg, 90 * deg});
  cv::Mat image = cv::Mat::zeros(4, 4, CV_8U);
  detector.setData(image);
  EXPECT_EQ(0, detector.getScore());
  EXPECT_EQ(0, detector.getPoints().size());

  image = cv::Mat::ones(4, 4, CV_8U);
  detector.setData(image);
  EXPECT_LT(0, detector.getScore());
  EXPECT_LT(0, detector.getPoints().size());

  image = cv::Mat::zeros(4, 4, CV_8U);
  detector.setData(image);
  EXPECT_EQ(0, detector.getScore());
  EXPECT_EQ(0, detector.getPoints().size());
}

TEST(ShapeDetector, matchesShape) {
  cv::Mat image = cv::Mat::ones(4, 4, CV_8U);
  ShapeDetector perfectMatch({ 90 * deg, 90 * deg, 90 * deg, 90 * deg});
  ShapeDetector someMatch({ 70 * deg, 70 * deg, 70 * deg, 70 * deg});
  ShapeDetector wrongNumSides({ 90 * deg, 90 * deg, 90 * deg, 90 * deg, 90 * deg});
  ShapeDetector noMatch({ -200 * deg, -200 * deg, -200 * deg, -200 * deg});

  perfectMatch.setData(image);
  someMatch.setData(image);
  wrongNumSides.setData(image);
  noMatch.setData(image);

  EXPECT_EQ(4, perfectMatch.getPoints().size());
  EXPECT_EQ(4, someMatch.getPoints().size());
  EXPECT_EQ(4, wrongNumSides.getPoints().size());
  EXPECT_EQ(0, noMatch.getPoints().size());

  EXPECT_GE(0, noMatch.getScore());
  EXPECT_LT(0, wrongNumSides.getScore());
  EXPECT_LT(wrongNumSides.getScore(), someMatch.getScore());
  EXPECT_LT(someMatch.getScore(), perfectMatch.getScore());
}

TEST(InRangeInstructions, threshSucceeds) {
  cv::Mat image(4, 4, CV_8UC3, cv::Scalar(255, 0, 0));
  InRangeInstructions thresh(cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255), 4);
  thresh.Thresh(image);
  EXPECT_EQ(1, image.channels());
  EXPECT_EQ(255, image.at<unsigned char>(0, 0));

  image=cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0));
  thresh.Thresh(image);
  EXPECT_EQ(1, image.channels());
  EXPECT_EQ(0, image.at<unsigned char>(0, 0));
}
