#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <stdio.h>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <array>

#include "StopSignDetection.cpp"
#include "SignDetection.hpp"
#include "countCars.hpp"
#include "leadCarScan.cpp"

using namespace std;
using namespace cv;

ShapeDetector ssd;
leadCarScan carScan;

int CountWhitePixels(Mat img);
string leadCarStatus(double areaOfContour);
Mat getInterval(Mat img, string color);
array<bool, 3> ShapeDetection(Mat img, bool VERBOSE, bool VIDEO , int BLUEINSIGN);
float getSensorData(float distanceMessage, int sendStamp, float &totalSum, int &counter, bool &gotNewDataFromLeft, int &falseCounter);
bool stopSignRed( vector<Point> contour,Mat img , bool VIDEO , int AMOUNTOFRED);

// GET ULTRASONIC/IR-SENSOR VALUES:
const int MAXCOUNT = 4; //was 3
float frontSensorData[MAXCOUNT];
float leftSensorData[MAXCOUNT];

int32_t main(int32_t argc, char **argv) {
     int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height"))) {
    cerr << argv[0] << " attaches to a shared memory area containing an ARGB image."<< endl;
    cerr << "Usage:   " << argv[0]  << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << endl;
    cerr << " --cid:    CID of the OD4Session to send and receive messages"<< endl;
    cerr << " --name:   name of the shared memory area to attach" << endl;
  } else {
        cluon::OD4Session od4{static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};
        const uint32_t WIDTH{static_cast<uint32_t>(stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(stoi(commandlineArguments["height"]))};
        const string NAME{commandlineArguments["name"]};
        unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const bool VIDEO{commandlineArguments.count("video") != 0};
        const uint16_t MISSEDSIGNS{static_cast<uint16_t>(stoi(commandlineArguments["missed"]))};
        const double MINAREA{static_cast<double>(stod(commandlineArguments["minarea"]))};
        const uint16_t LOOKLEFT{static_cast<uint16_t>(stoi(commandlineArguments["look"]))};
        const uint16_t AMOUNTOFRED{static_cast<uint16_t>(stoi(commandlineArguments["red"]))};
        const uint16_t BLUEINSIGN{static_cast<uint16_t>(stoi(commandlineArguments["blue"]))};
        const bool ENABLECALIBRATION{commandlineArguments.count("calib") != 0};


        if (sharedMemory && sharedMemory->valid()) { 
            clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << endl;
            
            int mode = 0; //should be 0

            od4.dataTrigger(2005, [&od4, &mode](cluon::data::Envelope &&envelope) {
              DriveMode currentDriveMode = cluon::extractMessage<DriveMode>(std::move(envelope));
              mode = currentDriveMode.mode();
            });

            countCars ccars;

            ////**COUNTING CARS:**////
            int amountOfCars = 0; // should be 0, only 3 for testing count passing cars
            int leftCar = 0;
            int frontCar = 0;
            int rightCar = 0;
            // ultrasonic:
            bool gotNewDataFromLeft;
            int falseCounter = 0;
            float frontTotalSum = 0;
            float leftTotalSum = 0;
            int frontCounter = 0;
            int leftCounter = 0;
            float frontSensorValue = 0.0;
            float leftSensorValue = 0.0;
            double areaOfContour = 0.0;
            
            bool runOnce  = true;
            bool leadCarSeen = false;
            int lookLeft = 0;
            
            string distance = "";
            std::array<bool, 3> trafficRules;
            TrafficRules trafficSignRules;
            opendlv::proxy::PedalPositionRequest pedalReq;

            leadCarScan leadCar;
            DriveMode driveMode;
            CalibrateSteering calibrateSteering;
            LeadCarDistance leadCarDistance;
            InstructionMode instructionMode;
            instructionMode.directionAllowed(false);
            //driveMode.directionInstruction(false); //drivemode initially set to false because we're not ready to receive instruction
            driveMode.atStopSign(false);
            od4.send(instructionMode);
            od4.send(driveMode);
            bool STOPSIGN_DETECTION = true;

            ssd.setMinArea(MINAREA);
            ssd.setNumberOfMissedSign(MISSEDSIGNS);

            while(od4.isRunning()){

                Mat img;
                sharedMemory->wait();
                sharedMemory->lock();
                {
                Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                img = wrapped.clone();
                }
                sharedMemory->unlock();

                Mat greenInputImage = getInterval(img, "green");
                Mat blackInputImage = getInterval(img, "black");
                Mat orangeInputImage = getInterval(img, "orange");
                
                if(mode == 0){
                    trafficRules = ShapeDetection(img, VERBOSE, VIDEO, BLUEINSIGN);
                    
                    //Follow lead car:
                    areaOfContour = carScan.findLeadCar(orangeInputImage , VIDEO);
                    if(areaOfContour > 1000){
                        leadCarSeen = true;
                        if(ENABLECALIBRATION){
                            calibrateSteering.CalibrateSteeringAngle(leadCar.CalibrateSteeringAngle(areaOfContour, orangeInputImage ,VERBOSE));
                            od4.send(calibrateSteering);
                        }
                        cout << "areaOfContour for the lead car: " << areaOfContour << endl;
                        distance = leadCarStatus(areaOfContour); 
                        leadCarDistance.distance(distance);
                        od4.send(leadCarDistance);
                    } 
                    
                    if(!leadCarSeen && lookLeft < LOOKLEFT){
                      //Find left car:
                      cout << "entered find left car" << endl;
                      leftCar = ccars.findCars(greenInputImage, 0, leftCar);
                      amountOfCars = leftCar;
                      lookLeft++;
                    }

                    Mat image(img);
                    Rect myROI(0, 0, img.size().width, img.size().height * 3 / 4);
                    Mat croppedImage = image(myROI);

                    Mat grayImg, frame_thresholdgreen;
                    cvtColor(croppedImage, grayImg, COLOR_BGR2GRAY);

                    if(STOPSIGN_DETECTION){

                      vector<Point> contour = ssd.getBiggestOctagon(grayImg);

                      bool red = stopSignRed(contour, img, VIDEO, AMOUNTOFRED);
                      
                      double contArea = ssd.getArea(contour); //was area

                      if(VERBOSE){
                        //cout << "Current Octagon area :" << area << endl;
                      }

                      if(ssd.Threshhold_reached == false){ //was area
                        ssd.setArea(contArea);
                        ssd.stopSignLogic(red);
                      }else{
                          cout << "-- Exiting stopsign detection." <<  " -- Failed frames: " << ssd.failed_frames << endl;
                          STOPSIGN_DETECTION = false;
                          cout << "at stop sign" << endl;

                          trafficSignRules.leftAllowed(trafficRules[0]);
                          trafficSignRules.forwardAllowed(trafficRules[1]);
                          trafficSignRules.rightAllowed(trafficRules[2]);
                          od4.send(trafficSignRules); 
                                                
                          driveMode.atStopSign(true);
                          od4.send(driveMode);
                      }
                    }

                    //TODO add linearacceleration
                    //TODO add calibration

                } else {  //2nd mode
                    if(runOnce){
                        cout << "ran FIND FRONT RIGHT cars" << endl;
                        frontCar = ccars.findCars(greenInputImage, 1, frontCar); //needs to run after car has stopped fully
                        rightCar = ccars.findCars(greenInputImage, 2, rightCar);  //needs to run after car has stopped fully
                        amountOfCars = leftCar + frontCar + rightCar;
                        runOnce = false;
                    }
                    if (amountOfCars == 0) {
                        //Allow car to drive out of intersection:
                        instructionMode.directionAllowed(true);
                        od4.send(instructionMode);
                    } else {
                        //  **Credit: --->some of this code is based on example_control code, start*
                        auto onDistanceReading{
                          [&od4, &gotNewDataFromLeft,&leftSensorValue, &frontSensorValue,&frontTotalSum, &frontCounter, &leftCounter,&leftTotalSum, &falseCounter](cluon::data::Envelope &&envelope) {
                            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                            const uint16_t senderStamp = envelope.senderStamp();
                            
                            gotNewDataFromLeft = false;
                              
                            if (senderStamp == 0) {
                                frontSensorValue = getSensorData(msg.distance(), 0, frontTotalSum, frontCounter,gotNewDataFromLeft, falseCounter);
                            } 
                            else if (senderStamp == 1) {
                                leftSensorValue = getSensorData(msg.distance(), 1, leftTotalSum, leftCounter,gotNewDataFromLeft, falseCounter);
                            }
                          }
                        };
                        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(),onDistanceReading);
                        //  **Credit: --->some of this code is based on example_control code, end*

                        // count how many cars pass by:
                        amountOfCars = ccars.countPassingCars(frontSensorValue, amountOfCars, 0, blackInputImage);
                        if (!gotNewDataFromLeft) {
                            falseCounter++;
                        }
                        if (falseCounter >= 5) { // needed for the left sensor
                            amountOfCars = ccars.countPassingCars(0, amountOfCars, 1, blackInputImage);
                        } else {
                            amountOfCars = ccars.countPassingCars(leftSensorValue, amountOfCars, 1, blackInputImage);
                        }
                    }

                }

            }
        }
    }
    return retCode;
}

string leadCarStatus(double areaOfContour){
  string distance;
  double minAcceptableArea = 3000; //need to look all these numbers up!
  double maxAcceptableArea = 3500;
  double minOutOfBounds = 200;
  if(areaOfContour >= minAcceptableArea && areaOfContour <= maxAcceptableArea){
      distance = "ok";
  }
  else if(areaOfContour < minAcceptableArea && areaOfContour > minOutOfBounds){
      distance = "too far";
  }
  else if(areaOfContour > maxAcceptableArea){
      distance = "too close";
  }
  return distance;
}

Mat getInterval(Mat img, string color){
  Mat hsvImg;
  cvtColor(img, hsvImg, COLOR_BGR2HSV);
  Mat intervalOutput;
  if (color == "black"){ //black
    inRange(hsvImg, Scalar(0, 0, 0), Scalar(180, 255, 30), intervalOutput);
  }
  if (color == "orange"){ //orange
    inRange(hsvImg, Scalar(2, 130, 154), Scalar(23, 166, 255), intervalOutput);
  }
  else if (color == "green"){ //green
    inRange(hsvImg, Scalar(30, 80, 125), Scalar(55, 255, 255), intervalOutput);
  }
  else if(color == "red"){ //red
    inRange(hsvImg, Scalar(149, 121, 131), Scalar(179, 198, 247), intervalOutput);
  }
  return intervalOutput;
}

float getSensorData(float distanceMessage, int sendStamp, float &totalSum, int &counter, bool &gotNewDataFromLeft, int &falseCounter) {
  float avg;
  if (sendStamp == 0) { // front sensor
    float frontValue = distanceMessage;
    if (counter < MAXCOUNT) {
      frontSensorData[counter] = distanceMessage;
      counter++;
    } else {
      for (int i = 0; i < MAXCOUNT; i++) {
        totalSum += frontSensorData[i];
      }
      avg = totalSum / MAXCOUNT;
      frontValue = avg;
      counter = 0;
      totalSum = 0;
    }
    return frontValue;
  }
  else if (sendStamp == 1) { // left sensor
    float leftValue = distanceMessage;
    gotNewDataFromLeft = true;
    falseCounter = 0;
    if (counter < MAXCOUNT) {
      leftSensorData[counter] = distanceMessage;
      counter++;
    } else {
      for (int i = 0; i < MAXCOUNT; i++) {
        totalSum += leftSensorData[i];
      }
      avg = totalSum / MAXCOUNT;
      leftValue = avg;
      counter = 0;
      totalSum = 0;
    }
    return leftValue;
  }
  return 1;
}

bool stopSignRed(vector<Point> contour , Mat img  , bool VIDEO, int  AMOUNTOFRED){
    Mat frame_thresholdred;
    Rect br = boundingRect(contour);
    Mat FullStopSign(img, br);
    frame_thresholdred = getInterval(FullStopSign, "red");
    int WhitePixelsInStopSign = CountWhitePixels(frame_thresholdred);

    cout << "area of red" << WhitePixelsInStopSign << endl;
    if(VIDEO && WhitePixelsInStopSign > AMOUNTOFRED){
      imshow("FullStopSign" , FullStopSign);
    }
    if(WhitePixelsInStopSign > AMOUNTOFRED){
      return true;
    }
    else{
      return false;
    }
}