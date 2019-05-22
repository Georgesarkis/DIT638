#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

using namespace cv;
using namespace std;

class leadCarScan {

public:
  double findLeadCar(const Mat &image);
};


//: return the area of contour of the lead car
double leadCarScan::findLeadCar(const Mat &image) {
  vector<vector<Point>> contours;
  findContours(image, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  Rect BigestBoundRect;
  double biggest_area = 0.0;
  vector<Point> bigbest_contour;

  double current_area = 0;
  vector<Point> approx;
  for (size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);
    current_area = contourArea(approx);
    if (current_area > biggest_area) {
      BigestBoundRect = boundingRect(approx);
      biggest_area = current_area;
      bigbest_contour = approx;
    }
  }
  return biggest_area;
}

