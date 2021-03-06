#include "SignDetection.hpp"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <array>
#include <stdio.h>
#include <string>

using namespace std;
using namespace cv;

// count number of times it seen to minimize false positives
int countRight = 0;
int countleft = 0;
int countForward = 0;
array<bool, 3> trafficSignArray = {true, true, true};


//: count white pixels in the image.
int CountWhitePixels(Mat img) {
  vector<Point> all_pixels;
  findNonZero(img, all_pixels);
  return all_pixels.size();
}

//: detect sign base on the blue area in the image.
void DetectBlueArea(Mat full_sign, bool VERBOSE , int BLUEINSIGN) {
  // Right part of the image
  Mat image1(full_sign);
  Rect myROI1(0, 0, full_sign.size().width / 2,
              full_sign.size().height * 3 / 4);
  Mat right_sign = image1(myROI1);

  int right_sign_width = right_sign.size().width;

  // Left part of the image
  Mat image2(full_sign);
  Rect myROI2(right_sign_width, 0, full_sign.size().width - right_sign_width,
              full_sign.size().height);
  Mat left_sign = image2(myROI2);

  // Top part of the image
  Mat image3(full_sign);
  Rect myROI3(full_sign.size().width / 2, 0, full_sign.size().width / 10,
              full_sign.size().height / 2);
  Mat top_sign = image3(myROI3);

  // white pixels in every image
  int WhiteInRight = CountWhitePixels(right_sign);
  int WhiteInLeft = CountWhitePixels(left_sign);
  int WhiteInTop = CountWhitePixels(top_sign);

  // Logic to comparing the white erea
  if (WhiteInLeft > WhiteInRight && WhiteInLeft - WhiteInRight > BLUEINSIGN && WhiteInLeft >= WhiteInTop &&  WhiteInRight + WhiteInLeft > 1000){
      trafficSignArray[2] = false;
      if(VERBOSE){
        cout << "==============can't turn right sign found==============" << endl;
      }
  } else if (WhiteInRight > WhiteInLeft && WhiteInRight - WhiteInLeft > BLUEINSIGN && WhiteInRight >= WhiteInTop && WhiteInRight + WhiteInLeft > 1000){
      trafficSignArray[0] = false;
      if(VERBOSE){
        cout << "==============can't turn left sign found==============" << endl;
      }
  }
}


//: Cut the upper right part of the image
Mat GetCroppedImage(Mat img) {
  Mat image(img);
  Rect myROI(img.size().width / 2, 0, img.size().width / 2, img.size().height * 3 / 4);
  Mat croppedImage2 = image(myROI);
  medianBlur(croppedImage2, croppedImage2, 3);
  return croppedImage2;
}

//: detect a square in the image
array<bool, 3> ShapeDetection(Mat img, bool VERBOSE, int BLUEINSIGN) {
  Mat bw, grayImg;
  Mat croppedImage2 = GetCroppedImage(img);
  cvtColor(croppedImage2, grayImg, COLOR_BGR2GRAY);
  blur(grayImg, grayImg, Size(3, 3));
  Canny(grayImg, bw, 80, 240, 3);
  vector<vector<Point>> contours2;
  vector<Point> approx2;

  findContours(bw.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  for (vector<Point>::size_type i = 0; i < contours2.size(); i++) {
    approxPolyDP(Mat(contours2[i]), approx2, arcLength(Mat(contours2[i]), true) * 0.02, true);
    if (fabs(contourArea(contours2[i])) < 100 || !isContourConvex(approx2))
      continue;
    if (approx2.size() == 4) {
      Rect br = boundingRect(contours2[i]);
      Mat full_sign(croppedImage2, br);

      Mat image1(full_sign);
      Rect myROI1((full_sign.size().width) / 10, (full_sign.size().height) / 10,
                  full_sign.size().width * 8 / 10,
                  full_sign.size().height * 9 / 10);
      Mat cut_image = image1(myROI1);
      cvtColor(cut_image, cut_image, COLOR_BGR2HSV);

      inRange(cut_image, Scalar(80, 0, 0), Scalar(130, 255, 255), cut_image);
      try {
        if(trafficSignArray[0] && trafficSignArray[2]){
          DetectBlueArea(cut_image, VERBOSE, BLUEINSIGN);
        }
      } catch (...) {
        if (VERBOSE)
          cout << "something bad happend" << endl;
      }
    }
  }
  return trafficSignArray;
}
