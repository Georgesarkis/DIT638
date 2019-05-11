#include "countCars.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;

int x;
int y;
int width;
int height;

bool carPassed = false;
bool carPassedLeft = false;
bool carPassedFront = false;
bool onpc = false;

bool enteredFront = false;
bool enteredRight = false;
bool enteredLeft = false;


int minTime = 60;
int timeCounter = minTime+1;
float previousSensorData = 0; //since we first check front sensor and then left sensor we can check this? to compare the front sensor if insideof bounds or not
//if in of bpunds we dont say false if out of bounds nothing is seen in front so we say false

bool carOnCamera = false;
int carFound;

std::vector<std::vector<cv::Point>> foundContours;
std::vector<std::vector<cv::Point>> findContoursInROI(int xC,int yC,int widthC,int heightC, Mat image);

// ****METHOD:
int countCars::findCars(Mat image, int side, int prevAmount){ //count the amount of contours in this left rectangle and return the output

    //either have the sides for roi here or let user input them as parameters
    if(side == 0){ //left side roi
        x = 0;
        y = 0;
        width = 130;
        height = 324;
    }
    else if(side == 1){ //middle/front roi
        x = 0;
        y = 0;
        width = 260;
        height = 270;
    }
    else if(side == 2){ //right side roi
        x = 400;
        y = 0;
        width = 233;
        height = 324;
    }
    else{
        throw invalid_argument("no allowed side found");
    }

    foundContours = findContoursInROI(x, y, width, height, image);

    if(onpc){
        //imshow("Image", image);  //inserted this here now, used to be at the bottom tho
        //waitKey(1);
    }

    if(foundContours.size() >= 1){  //should remake this in a function for less code repetition maybe (can wait for later tho)
        for(size_t i = 0; i < foundContours.size(); i++) {
            //cout << "           contour area: " << contourArea(foundContours[i]) <<endl;
            if(side == 0 && !enteredLeft && prevAmount == 0){ //left cars
                if(contourArea(foundContours[i]) > 250 && contourArea(foundContours[i]) < 2000){
                    cout << "----->left car found" << endl;
                    enteredLeft = true;
                    return 1;
                }
                if(i == foundContours.size()){
                    enteredFront = true;
                }
            }
            if(side == 1 && !enteredFront && prevAmount == 0){ //front cars
                if(contourArea(foundContours[i]) > 450 && contourArea(foundContours[i]) < 650){ //change these numbers!!!
                    cout << "----->front car found" << endl;
                    enteredFront = true;
                    return 1;
                }
                if(i == foundContours.size()){
                    enteredFront = true;
                }
            }
            if(side == 2 && !enteredRight && prevAmount == 0){ //right cars  //for this we should move the green note forward on the car
                if(contourArea(foundContours[i]) > 450 && contourArea(foundContours[i]) < 650){ //change these numbers!!!
                    cout << "----->right car found" << endl;
                    enteredRight = true;
                    return 1;
                }
                if(i == foundContours.size()){
                    enteredRight = true;                    
                }
            }      
        }
    }
    return prevAmount;
}

// ****METHOD:
int countCars::countPassingCars(float sensorDistance, int currentAmountOfCars, int sensorType, Mat image){ //this input should be the calculated median value or w.e

    carFound = 0;

    if(timeCounter > 500){
        timeCounter = minTime+1;
    }

    //cout << ">>>>TC amount: " << timeCounter << endl;
    if(sensorType == 0){ //front ultrasonic
        if(sensorDistance <= 0.65 && sensorDistance >= 0.03){       //make these variables
            if(currentAmountOfCars > 0  && !carPassed){
                currentAmountOfCars--;
                cout << "       one car passed FROOOOOOOONT at DISTANCE: " << sensorDistance << endl;
                carPassed = true;
                timeCounter = 0;
            }
        }
    }
    else if(sensorType == 1){ //left-side ir sensor
        //cout << "left SENSOR: " << sensorDistance << endl;
        if(sensorDistance <= 0.31 && sensorDistance >= 0.045){        //make these variables
            if(currentAmountOfCars > 0 && !carPassed){
                if(timeCounter < minTime){
                    cout << "DONT count this car" << endl; //maybe set carpassed = true here för att den inte ska fortsätta räkna varje vi ser som ny
                    carPassed = true;  //added this
                }
                else{
                    cout << "       one car passed LEEEEEEEEEEEEFT side at DISTANCE: " << sensorDistance << endl;
                    currentAmountOfCars--;
                    carPassed = true;
                }
            }
        }
    }
    
    if(!carPassed){ //need to check again that this is false because we dont want to go into this code if its true
        //maybe need to put this below here also in an if statement because if the front car driving right
        //does get picked up by the sensor, we DONT also want to pick it up with the camera because we get wrong number of cars passed
        x = 0;
        y = 285;
        width = 210;
        height = 60;

        foundContours = findContoursInROI(x, y, width, height, image);

        if(foundContours.size() >= 1){
            for(size_t i = 0; i < foundContours.size(); i++) {
                //cout << "contour: " << contourArea(foundContours[i]) << endl;
                if(contourArea(foundContours[i]) > 100 && contourArea(foundContours[i]) < 650){
                    if(currentAmountOfCars > 0){
                        currentAmountOfCars--;
                        cout << "       1 car passed CAMERA" << endl;
                        carPassed = true;
                        carOnCamera = true;
                        carFound = 1;
                    }
                }
            }
            if(carFound == 0){            //might be overkill with all these things and there is a simpler way
                carOnCamera = false;
                carPassed = false;
            }
        }
        else{
            carOnCamera = false; //need this somewhere else too! maybe dont need to set it here since its set above
            carPassed = false;
        }
    }
    
    if((sensorDistance > 0.65 || sensorDistance < 0.03) && (previousSensorData > 0.65 || previousSensorData < 0.03) && !carOnCamera && currentAmountOfCars > 0){
        carPassed = false;
        //cout << "       ***no car***" << endl; //stops showing up after having counted 3 cars
    }
    if(carPassed && currentAmountOfCars > 0){
        //cout << "current data: "  << sensorDistance << ". prev data: " << previousSensorData << " carPassed = " << carPassed << endl;
    }

    timeCounter++;

    previousSensorData = sensorDistance;

    return currentAmountOfCars;
}


// ****METHOD:
std::vector<std::vector<cv::Point>> countCars::findContoursInROI(int xC,int yC,int widthC,int heightC, Mat image){
    std::vector<std::vector<cv::Point>> contours;

    Rect roiAreaRectangle(xC,yC,widthC,heightC);
    Mat roiImage = image(roiAreaRectangle);                     //get the region of interest of original image

    rectangle(image, roiAreaRectangle, Scalar(255), 1, 8, 0);   //draw the rectangle for the area on the original picture


    cv::findContours(roiImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    std::vector<std::vector<cv::Point>> contoursPoly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());

    for(size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP( contours[i], contoursPoly[i], 3, true);
        boundRect[i] = cv::boundingRect( contoursPoly[i] );
    }

    if(onpc){
        cv::imshow("Contour", image);
        cv::waitKey(1);
    }

    return contours;
}

//we can also like compare images, like do some object detection of an empty intersection
//and then when its not empty we can tell something is driving there and we remove it from the list

/*
    counting passing cars,
        easy cases:
            car on the right drives forward,    (ultrasonic)
            car on the right turns left         (ir sensor)
            car on the left turns right         (ir sensor)
            car on the left drives forward      (ultrasonic)
            car in front drives forward         (ir sensor)

        medium cases: (might not be able to catch with sensor)
            car in front drives left            (ultrasonic)    OK
            car on the left turns left          (ultrasonic)    OK
            car on right turns right            (ultrasonic)    OK

        difficult cases: (wont be reached by sensor)
            car in front drives right
                for this we can set up a small roi on the left part of the intersection
                and try to detect a big amount of black, and when we do we
                remove a car from the queue

                looking at the vids tho this wont always work if the car driver doesnt clearly enter the intersection
                and right away drives to the side, then the car might not enter the roi
*/
