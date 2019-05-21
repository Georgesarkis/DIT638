#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

using namespace cv;
using namespace std;

class ShapeDetector {
public:

  bool Threshhold_reached = false;
  int failed_frames = 0;

  string detect(const vector<Point> &input);
  Point getCenter(const vector<Point> &input);
  double getArea(const vector<Point> &input);
  vector<Point> getBiggestOctagon(const Mat &image);
  void showAllShapes(const Mat &image);
  bool stopSignInRange(const double &input);
  void setMinArea(double input);
  void setNumberOfMissedSign(int numberOfMissedSigns);
  bool lookForStopSign(bool red);
  bool followStopsign();
  bool stopSignLogic(bool red);
  void setArea(double &input);

};

double minStopSignArea = 1000;
int number_of_missed_signs = 0;
int max_number_of_missed_signs = 30;

bool followingStopSign = false;

double area;
void ShapeDetector::setArea(double &input){
  area = input;
}

void ShapeDetector::setMinArea(double input){
  minStopSignArea = input;
}

void ShapeDetector::setNumberOfMissedSign(int numberOfMissedSigns){
  max_number_of_missed_signs = numberOfMissedSigns;
}

double ShapeDetector::getArea(const vector<Point> &input) {
  try {
    return contourArea(input);
  } catch (...) {
    failed_frames++;
  }
  return 0.0;
}

//: Looks for the stop sign
bool ShapeDetector::lookForStopSign(bool red){
  if(area > minStopSignArea && red){
    cout << "----- Found a stopsign! - Starting to follow it ------";
    followingStopSign = true;
  }
  return true;
}

bool ShapeDetector::followStopsign(){
  //: Counts the number of times we haven't seen the stopsign
  if(area == 0.0){
    cout << "Failed frame." << endl;
  }else if(area >= minStopSignArea){
    number_of_missed_signs = 0;
    //cout << "-- Stop sign found , reseting counter ---- " << endl;
  }else if(area < minStopSignArea){
    number_of_missed_signs++;
    cout << "Not seeing stop sign . Frames not seen : " << number_of_missed_signs << endl;
  }

  //: Returns true ( signaling that we are still following a stopsign) if we haven't seen it in a while
  if(number_of_missed_signs >= max_number_of_missed_signs){
    Threshhold_reached = true;
    cout << "-- Threshhold reached , stopping car ---- " << endl;
    return false;
  }else{
    return true;
  }
}

//: logic for determinging if we are following the stopsign
bool ShapeDetector::stopSignLogic(bool red) {

  if(followingStopSign){
    return followStopsign();
  }else{
    return lookForStopSign(red);
  }

}

Point ShapeDetector::getCenter(const vector<Point> &input) {
  Moments M = moments(input);
  int X = static_cast<int>(M.m10 / M.m00);
  int Y = static_cast<int>(M.m01 / M.m00);
  return Point(X, Y);
}



string ShapeDetector::detect(const vector<Point> &input) {
  Mat curve = Mat(input);

  string shape = "unidentified";
  Mat approx;
  approxPolyDP(curve, approx, 0.04 * arcLength(curve, true), true); // 0.01~0.05
  const int num_of_vertices = approx.rows;

  if (num_of_vertices == 3) {
    shape = "triangle";
  } else if (num_of_vertices == 4) {

    Rect rec = boundingRect(approx);
    double ar = 1.0 * rec.width / rec.height;

    if (ar >= 0.95 && ar <= 1.05) {
      shape = "square";
    } else {
      shape = "rectangle";
    }

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
  } else if (num_of_vertices == 5) {
    shape = "decagon";
  }
  return shape;
}

vector<Point> ShapeDetector::getBiggestOctagon(const Mat &image) {

  Mat blured_image;

  blur(image, blured_image, Size(3, 3));
  Canny(image, blured_image, 80, 240, 3);

  vector<vector<Point>> contours;
  findContours(blured_image, contours, CV_RETR_EXTERNAL,
               CV_CHAIN_APPROX_SIMPLE);

  double biggest;
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

  return bigC;
}

void ShapeDetector::showAllShapes(const Mat &image) {

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