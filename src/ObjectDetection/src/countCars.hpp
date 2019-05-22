#ifndef COUNTCARS_HPP

#define COUNTCARS_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class countCars {

  int x; 
  int y;
  int width;
  int height;

  bool carPassed;

public:
  int findCars(cv::Mat image, int side, int prevAmount);
  int countPassingCars(float sensorDistance, int currentAmountOfCars, int sensorType, cv::Mat image);
  std::vector<std::vector<cv::Point>> findContoursInROI(int xC, int yC, int widthC, int heightC, cv::Mat image);
};

#endif