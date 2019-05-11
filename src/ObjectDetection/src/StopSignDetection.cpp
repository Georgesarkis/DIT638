/*

Inspiration taken from : http://www.voidcn.com/article/p-knbavvdq-bnu.html

Written by : Jacob Olsson

*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

using namespace cv;
using namespace std;

class StopSignDetection {

  bool VERBOSE = false;

  double minStopSignArea = 1000;
  int number_of_missed_signs = 0;
  int max_number_of_missed_signs = 4;

  bool STOPSIGN_FOUND = false;
  double area = 0;
  vector<Point> current_contour{};

  double getBiggestOctagon(Mat image);
  string detect(vector<Point> input);
  void followStopsign();
  void lookForStopSign();
  double getArea(vector<Point> input);
  Point getCenter(vector<Point> input);
  bool same(double a, double b);

public:
  bool Threshhold_reached = false;
  int failed_frames = 0;
  void setNumberOfMissedSign(int numberOfMissedSigns);
  void setMinArea(double input);
  void setArea(double input);

  void showAllShapes(Mat image);
  void run(Mat image, bool verbose, bool VIDEO);
};

void StopSignDetection::run(Mat image, bool verbose, bool VIDEO) {

  VERBOSE = verbose;

  area = getBiggestOctagon(image);

  if (STOPSIGN_FOUND) {
    followStopsign();
  } else {
    lookForStopSign();
  }

  if (VIDEO) {
    Mat drawing = image.clone();

    Point pt = getCenter(current_contour);

    putText(drawing, "octagon", pt, FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(255, 255, 255), 2);
    Scalar color = Scalar(0, 0, 255);
    polylines(drawing, current_contour, true, color, 1, 8);

    imshow("StopSignDetection Vision", drawing);
    waitKey(1);
  }
}

void StopSignDetection::setArea(double input) { area = input; }

void StopSignDetection::setMinArea(double input) { minStopSignArea = input; }

void StopSignDetection::setNumberOfMissedSign(int numberOfMissedSigns) {
  max_number_of_missed_signs = numberOfMissedSigns;
}

Point StopSignDetection::getCenter(vector<Point> input) {
  Moments M = moments(input);
  int X = static_cast<int>(M.m10 / M.m00);
  int Y = static_cast<int>(M.m01 / M.m00);
  return Point(X, Y);
}

double StopSignDetection::getArea(vector<Point> input) {
  try {
    return contourArea(input);
  } catch (...) {
    failed_frames++;
  }
  return 0.0;
}

//: Looks for the stop sign
void StopSignDetection::lookForStopSign() {
  if (area > minStopSignArea) {
    cout << "----- Found a stopsign! - Starting to follow it ------";
    STOPSIGN_FOUND = true;
  }
}

//: function done by "Daniel Laügt" - Feb 6 '16 at 18:29
bool StopSignDetection::same(double a, double b) {
  return std::nextafter(a, std::numeric_limits<double>::lowest()) <= b &&
         std::nextafter(a, std::numeric_limits<double>::max()) >= b;
}

void StopSignDetection::followStopsign() {
  //: Counts the number of times we haven't seen the stopsign
  if (same(area, 0.0)) {
    // cout << "Failed frame." << endl;
  } else if (area >= minStopSignArea) {
    number_of_missed_signs = 0;
    // cout << "-- Stop sign found , reseting counter ---- " << endl;
  } else if (area < minStopSignArea) {
    number_of_missed_signs++;
    //  cout << "Not seeing stop sign . Frames not seen : "
    //       << number_of_missed_signs << endl;
  }

  //: Returns true ( signaling that we are still following a stopsign) if we
  // haven't seen it in a while
  if (number_of_missed_signs >= max_number_of_missed_signs) {
    StopSignDetection::Threshhold_reached = true;
    //  cout << "-- Threshhold reached , stoping car ---- " << endl;
  }
}

string StopSignDetection::detect(vector<Point> input) {

  string shape = "unidentified";
  const int num_of_vertices = input.size();

  if (num_of_vertices == 3) {
    shape = "triangle";
  } else if (num_of_vertices == 4) {
    shape = "rectangle";
  } else if (num_of_vertices == 5) {
    shape = "pentagon";
  } else if (num_of_vertices == 6) {
    shape = "hexagon";
  } else if (num_of_vertices == 7) {
    shape = "heptagon";
  } else if (num_of_vertices == 8) {
    shape = "octagon";
  } else if (num_of_vertices == 9) {
    shape = "nonagon";
  } else if (num_of_vertices == 10) {
    shape = "decagon";
  }
  return shape;
}

double StopSignDetection::getBiggestOctagon(Mat image) {

  Mat blured_image;
  blur(image, blured_image, Size(3, 3));
  Canny(blured_image, blured_image, 80, 240, 3);

  vector<vector<Point>> contours;
  findContours(blured_image, contours, CV_RETR_EXTERNAL,
               CV_CHAIN_APPROX_SIMPLE);

  double biggest = 0.0;
  vector<Point> bigC;

  string shape;
  double size;
  vector<Point> approx;

  for (size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(Mat(contours[i]), approx,
                 arcLength(Mat(contours[i]), true) * 0.02, true);

    shape = detect(approx);
    size = getArea(approx);
    if ((shape == "octagon") && (size > biggest)) {
      biggest = size;
      bigC = approx;
    }
  }
  current_contour = bigC;
  return biggest;
}

void StopSignDetection::showAllShapes(Mat image) {

  Mat blured_image;

  blur(image, blured_image, Size(3, 3));
  Canny(image, blured_image, 80, 240, 3);

  vector<vector<Point>> contours;
  findContours(blured_image, contours, CV_RETR_EXTERNAL,
               CV_CHAIN_APPROX_SIMPLE);

  string shape;
  Point pt;
  for (size_t i = 0; i < contours.size(); i++) {
    shape = detect(contours[i]);

    if (shape != "unidentified") {
      pt = getCenter(contours[i]);

      putText(image, shape, pt, FONT_HERSHEY_SIMPLEX, 0.5,
              Scalar(255, 255, 255), 2);
      Scalar color = Scalar(0, 0, 255);
      polylines(image, contours[i], true, color, 1, 8);
    }
  }
}
