#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

using namespace cv;
using namespace std;

Point MiddleOfRectangle;

class leadCarScan {
private:
  Mat getInterval(const Mat &img, int color);
  Point getCenter(vector<Point> input);
  double CalibrateSteering(cv::Mat img, cv::Point point, bool VERBOSE);

public:
  double findLeadCar(const Mat &image, bool VIDEO);
  double CalibrateSteeringAngle(double areaOfContour, Mat img, bool VERBOSE);
};

//: Get's the center of a contour
Point leadCarScan::getCenter(vector<Point> input) {
  Moments M = moments(input);
  int X = static_cast<int>(M.m10 / M.m00);
  int Y = static_cast<int>(M.m01 / M.m00);
  return Point(X, Y);
}

double leadCarScan::findLeadCar(const Mat &image, bool VIDEO) {
  vector<vector<Point>> contours;
  findContours(image, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  Rect BigestBoundRect;
  double biggest_area = 0.0;
  vector<Point> bigbest_contour;

  double current_area = 0;
  vector<Point> approx;
  for (size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(Mat(contours[i]), approx,
                 arcLength(Mat(contours[i]), true) * 0.02, true);
    current_area = contourArea(approx);
    if (current_area > biggest_area) {
      BigestBoundRect = boundingRect(approx);
      biggest_area = current_area;
      bigbest_contour = approx;
    }
  }

  //: For calibarating the wheels
  Scalar red = Scalar(0, 0, 255);
  Scalar green = Scalar(0, 255, 0);
  rectangle(image, BigestBoundRect.tl(), BigestBoundRect.br(), red, 2, 4, 0);
  MiddleOfRectangle =
      Point(BigestBoundRect.tl() * 0.5 + BigestBoundRect.br() * 0.5);
  circle(image, MiddleOfRectangle, 3, green);

  //: for debugin , shows the vision of the leadCar scanner
  if (VIDEO) {
    Mat drawing = image.clone();
    Point pt = getCenter(bigbest_contour);

    putText(drawing, "Lead Car", pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0),
            2);
    Scalar color = Scalar(255, 0, 0);
    polylines(drawing, bigbest_contour, true, color, 1, 8);

    imshow("Lead Car Vision", drawing);
    waitKey(1);
  }

  return biggest_area;
}

double leadCarScan::CalibrateSteeringAngle(double areaOfContour, Mat img,
                                           bool VERBOSE) {
  double Angle;
  if (areaOfContour > 1000) {
    return CalibrateSteering(img, MiddleOfRectangle, VERBOSE);
  }
  return Angle = 0;
}

double leadCarScan::CalibrateSteering(cv::Mat img, cv::Point point,
                                      bool VERBOSE) {
  double leftAngle = 0.11;
  double rightAngle = -0.11;

  if (point.x < (img.size().width / 2) - 50) {
    if (VERBOSE)
      cout << "turn car to left" << endl;
    return leftAngle;
  } else if (point.x > (img.size().width / 2) + 50) {
    if (VERBOSE)
      cout << "turn car to right" << endl;
    return rightAngle;
  }
  return 0;
}
