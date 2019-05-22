#include "countCars.hpp"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int x;
int y;
int width;
int height;

bool carPassed = false;
bool carPassedLeft = false;
bool carPassedFront = false;

bool enteredFront = false;
bool enteredRight = false;
bool enteredLeft = false;

int minTime = 60;
int timeCounter = minTime + 1;
int camTimeCounter = minTime + 1;
int leftTimeCounter = minTime + 1;
float previousSensorData = 0; 

int carReallyPassed = 0;

bool carOnCamera = false;
int carFound;

vector<vector<Point>> foundContours;
vector<vector<Point>> findContoursInROI(int xC, int yC, int widthC, int heightC, Mat image);

// ****METHOD: counts how many cars are in front of our car in the intersection by splitting images into different regions of interest
int countCars::findCars(Mat image, int side, int prevAmount) {
  
  // determine which area of the image we are interested in
  if (side == 0) { // left side roi
    x = 0;
    y = 0;
    width = 150; 
    height = 324;
  } else if (side == 1) { // front roi
    x = 0;
    y = 0;
    width = 260;
    height = 270;
  } else if (side == 2) { // right side roi
    x = 385; 
    y = 0;
    width = 255; 
    height = 324; 
  } else {
    throw invalid_argument("no allowed side found");
  }

  // Get contours for the image
  foundContours = findContoursInROI(x, y, width, height, image);

  // Check if any contour is of acceptable size, if so then return 1 so signal one car has been found
  if (foundContours.size() >= 1) {
    for (size_t i = 0; i < foundContours.size(); i++) {
      if (side == 0 && !enteredLeft && prevAmount == 0) { // left cars
        if (contourArea(foundContours[i]) > 100 && contourArea(foundContours[i]) < 2000) {
          cout << "----->left car found" << endl;
          enteredLeft = true;
          return 1;
        }
        if (i == foundContours.size()) {
          enteredFront = true;
        }
      }
      if (side == 1 && !enteredFront && prevAmount == 0) { // front cars
        if (contourArea(foundContours[i]) > 880 && contourArea(foundContours[i]) < 2200) { 
          cout << "----->front car found at " << contourArea(foundContours[i]) << endl;
          enteredFront = true;
          return 1;
        }
        if (i == foundContours.size()) {
          enteredFront = true;
        }
      }
      if (side == 2 && !enteredRight && prevAmount == 0) { // right cars 
        if (contourArea(foundContours[i]) > 740 && contourArea(foundContours[i]) < 3500) { 
          cout << "----->right car found at " << contourArea(foundContours[i]) << endl;
          enteredRight = true;
          return 1;
        }
        if (i == foundContours.size()) {
          enteredRight = true;
        }
      }
    }
  }
  return prevAmount;
}

// ****METHOD: takes sensor data and an image to check if a car has passed in front of our car or entered the intersection
int countCars::countPassingCars(float sensorDistance, int currentAmountOfCars, int sensorType, Mat image) {

  carFound = 0; // for the camera

  // make sure time counters dont become too large
  //time counters are used to prevent the same car caught by several sensors and camera to be counted as several cars
  if (timeCounter > 500) {
    timeCounter = minTime + 1;
  }
  if (camTimeCounter > 500) {
    camTimeCounter = minTime + 1;
  }
  if (leftTimeCounter > 500) {
    leftTimeCounter = minTime + 1;
  }

  //check if a car passed the front sensor, if it did we reset the time counter and until the count reaches 60 we wont increase the number
  //of passed cars even if another sensor or camera noticed the car
  if (sensorType == 0) { // front ultrasonic
    if ((sensorDistance < 0.50 && sensorDistance >= 0.03) || (sensorDistance > 0.61 && sensorDistance < 0.7)) { 
      
      carReallyPassed++;

      if (currentAmountOfCars > 0 && !carPassed && carReallyPassed >= 3) {
        if (camTimeCounter < minTime || timeCounter < minTime) {
          //cout << "***FRONT DONT count this car" << endl;
        } 
        else {
          timeCounter = 0;
            currentAmountOfCars--;
            cout << "       one car passed FROOOOOOOONT at DISTANCE: " << sensorDistance << endl;
        }
        carPassed = true;
      }
    }
    else{
        carReallyPassed = 0;
    }  
  } 
  else if (sensorType == 1) { // left-side ir sensor
    if (sensorDistance <= 0.31 && sensorDistance >= 0.045) { 
      if (currentAmountOfCars > 0 && !carPassed) {
        if (timeCounter < minTime || camTimeCounter < minTime || leftTimeCounter < minTime) {
          carPassed = true;
        } 
        else {
          cout << "       one car passed LEEEEEEEEEEEEFT side at DISTANCE: " << sensorDistance << endl;
          currentAmountOfCars--;
          carPassed = true;
          leftTimeCounter = 0;
        }
      }
    }
  }

  x = 0;
  y = 285;
  width = 260; 
  height = 65;

  // Get contours
  foundContours = findContoursInROI(x, y, width, height, image);

  //Go through the contours, if any are of acceptable size and the timeCounter is not less than 60 we count it as a car passing
  if (foundContours.size() >= 1) {
    for (size_t i = 0; i < foundContours.size(); i++) {
      if (contourArea(foundContours[i]) > 20 && contourArea(foundContours[i]) < 650) {
        if (currentAmountOfCars > 0 && !carPassed) {
          if (camTimeCounter > minTime && timeCounter > minTime) {
            currentAmountOfCars--;
            cout << "       1 car passed CAMERA at area " << contourArea(foundContours[i]) << endl;
            camTimeCounter = 0;
          }
          carPassed = true;
          carOnCamera = true;
        }
        carFound = 1;
      }
    }
    if (carFound == 0) {
      carOnCamera = false;
      carPassed = false;
    }
  }

  //if we get too big or too small sensor data we set car passed to false to allow other cars to pass
  if ((sensorDistance > 0.7 || sensorDistance < 0.03) && (previousSensorData > 0.65 || previousSensorData < 0.03) && !carOnCamera && currentAmountOfCars > 0) {
    carPassed = false;
  }

  timeCounter++;
  camTimeCounter++;
  leftTimeCounter++;

  previousSensorData = sensorDistance;

  return currentAmountOfCars;
}

// ****METHOD: 
vector<vector<Point>> countCars::findContoursInROI(int xC, int yC, int widthC, int heightC, Mat image) {
  vector<vector<Point>> contours;

  Rect roiAreaRectangle(xC, yC, widthC, heightC);
  Mat roiImage = image(roiAreaRectangle); // get the region of interest of original image

  rectangle(image, roiAreaRectangle, Scalar(255), 1, 8, 0); // draw the rectangle for the area on the original picture

  findContours(roiImage, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  vector<vector<Point>> contoursPoly(contours.size());

  vector<Rect> boundRect(contours.size());

  for (size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(contours[i], contoursPoly[i], 3, true);
    boundRect[i] = boundingRect(contoursPoly[i]);
  }
  return contours;
}