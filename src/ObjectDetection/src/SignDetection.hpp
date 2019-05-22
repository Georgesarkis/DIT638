#ifndef SIGNDETECTION_HPP

#define SIGNDETECTION_HPP

#include <array>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class SignDetection {

public:
  std::array<bool, 3> ShapeDetection(cv::Mat img, bool VERBOSE, bool VIDEO, int BLUEINSIGN);
  cv::Mat GetCroppedImage(cv::Mat img);
  int CountWhitePixels(cv::Mat img);
};

#endif
