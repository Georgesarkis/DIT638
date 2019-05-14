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

  double minStopSignArea = 1000;
  int number_of_missed_signs = 0;
  int max_number_of_missed_signs = 30;

  bool STOPSIGN_FOUND = false;
  double area = 0;
  int CountWhitePixels(Mat img);
  void getBiggestOctagon(const Mat &image , int size);
  string detect(const vector<Point> &input);
  void followStopsign();
  void lookForStopSign();
  double getArea(const vector<Point> &input);
  Point getCenter(const vector<Point> &input);
  bool same(double a, double b);

public:
  bool Threshhold_reached = false;
  int failed_frames = 0;
  void setNumberOfMissedSign(int numberOfMissedSigns);
  void setMinArea(double input);
  void setArea(double &input);

  void showAllShapes(const Mat &image);
  void run(const Mat &image, bool VERBOSE, bool VIDEO , int size);
};

void StopSignDetection::run(const Mat &image, bool VERBOSE, bool VIDEO , int size) {
  cout << "RUNNING SSD" << endl;
  // Setup a rectangle to define your region of interest
  Rect myROI1(0, 0, image.size().width, image.size().height * 3 / 4);
  // Crop the full image to that image contained by the rectangle myROI
  Mat croppedImage1 = image(myROI1);

  Rect myROI2(croppedImage1.size().width / 2, 0, croppedImage1.size().width / 2,croppedImage1.size().height);
  Mat croppedImage2 = image(myROI2);

  cout << "cut the image for stop sign" << endl;

  Mat img;
  cvtColor(croppedImage2, img, COLOR_BGR2GRAY);
  
  cout << "get the gray image" << endl;
  
  getBiggestOctagon(img , size);
  //area = getArea(contour);

  cout << "after calling funcation get biggest octagon" << endl;
  if (VERBOSE) {
    //cout << "Current Octagon area :" << area << endl;
  }

  if (STOPSIGN_FOUND) {
    cout << "STOP SIGN FOUND" << endl;
    followStopsign();
    cout << "after calling funcatrionfollowstopsign();" << endl;
  } /*else {
  //  lookForStopSign();
  }*/
  cout << "NO STOP SIGN SEEN" << endl;

  // Display image
  if (VIDEO) {
    /*
    Mat drawing = img.clone();
    if (area >= minStopSignArea) {
      Point pt = getCenter(contour);

      putText(drawing, "octagon", pt, FONT_HERSHEY_SIMPLEX, 0.5,
              Scalar(255, 255, 255), 2);
      Scalar color = Scalar(0, 0, 255);
      polylines(drawing, contour, true, color, 1, 8);
    }
    */
    imshow("StopSignDetection Vision", img);
    waitKey(1);
  }
}

void StopSignDetection::setArea(double &input) { area = input; }

void StopSignDetection::setMinArea(double input) { minStopSignArea = input; }

void StopSignDetection::setNumberOfMissedSign(int numberOfMissedSigns) {
  max_number_of_missed_signs = numberOfMissedSigns;
}

Point StopSignDetection::getCenter(const vector<Point> &input) {
  Moments M = moments(input);
  int X = static_cast<int>(M.m10 / M.m00);
  int Y = static_cast<int>(M.m01 / M.m00);
  return Point(X, Y);
}

double StopSignDetection::getArea(const vector<Point> &input) {
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
    cout << "----- Found a stopsign! - Starting to follow it ------" << endl;
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
    //cout << "Failed frame." << endl;
  } else if (area >= minStopSignArea) {
    number_of_missed_signs = 0;
    //cout << "-- Stop sign found , reseting counter ---- " << endl;
  } else if (area < minStopSignArea) {
    number_of_missed_signs++;
    //cout << "Not seeing stop sign . Frames not seen : "
     //    << number_of_missed_signs << endl;
  }

  //: Returns true ( signaling that we are still following a stopsign) if we
  //haven't seen it in a while
  if (number_of_missed_signs >= max_number_of_missed_signs) {
    StopSignDetection::Threshhold_reached = true;
  //  cout << "-- Threshhold reached , stoping car ---- " << endl;
  }
}

string StopSignDetection::detect(const vector<Point> &input) {

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
  } else if (num_of_vertices == 5) {
    shape = "decagon";
  }
  return shape;
}

void StopSignDetection::getBiggestOctagon(const Mat &image , int size) {
  cout << "after finding contours" << endl;
  vector<Point> approx;
  cout << "before for loop ye" << endl;
  
  Mat hsvimg;
  Mat intervalOutput;
  cout << "after iniit vars" << endl;
  cvtColor(image, hsvimg, COLOR_BGR2HSV);
  cout << "after cvtColor hsv" << endl;
  inRange(hsvimg, Scalar(2, 143, 136), Scalar(7, 255, 255), intervalOutput);

  cout << "after the changingcolor to red" << endl;


  vector<vector<Point>> contours;
  cout << "before first find contours" << endl;
  findContours(intervalOutput, contours, RETR_TREE,CHAIN_APPROX_SIMPLE);

  cout << "finding the couunt of the red color" << endl;


  vector<vector<Point>> contoursPoly(contours.size());
  cout << "after contours poly" << endl;
  vector<Rect> boundRect(contours.size());
  cout << "after init boundrect of first contours size" << endl;

  for (size_t i = 0; i < contours.size(); i++) {
    cout << "in the first for loop" << endl;

    approxPolyDP(contours[i], contoursPoly[i], 3, true);
    cout << "after approximating polys" << endl;
    boundRect[i] = boundingRect(contoursPoly[i]);
      cout << "after creating a rec around every red object" << endl;

    if (contourArea(contours[i]) > size) {
      cout << "found red big enough to check shapes for" << endl;
      Mat fullStopSign(image, boundRect[i]);
     cout << "cuting the image base on the rect" << endl;

      Mat blured_image;

      blur(fullStopSign, blured_image, Size(3, 3));
      cout << "BLURRED IMAGE" << endl;
      Canny(blured_image, blured_image, 80, 240, 3);
      cout << "after canny whatever the hell that does" << endl;

      vector<vector<Point>> contours1;
      findContours(blured_image, contours1, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        cout << "finding contours1 from the cutten image" << endl;

      for (size_t i = 0; i < contours1.size(); i++) {
        cout << "in second for loop" << endl;
        approxPolyDP(Mat(contours1[i]), approx,arcLength(Mat(contours1[i]), true) * 0.02, true);
        cout << "IM FABULOUS" << endl;
        
        if(approx.size() == 8){
          cout << "found octagone in the red shae cutten image" << endl;
          STOPSIGN_FOUND = true;
        }
      }
    }
  }
}

int StopSignDetection::CountWhitePixels(Mat img) {
  vector<Point> all_pixels;
  findNonZero(img, all_pixels);
  return all_pixels.size();
}

void StopSignDetection::showAllShapes(const Mat &image) {

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