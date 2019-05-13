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

using namespace std;
using namespace cv;

StopSignDetection ssd;
Mat getInterval(Mat img, string color);
array<bool, 3> ShapeDetection(Mat img, bool VERBOSE, bool VIDEO);
float getSensorData(float distanceMessage, int sendStamp, float &totalSum, int &counter, bool &gotNewDataFromLeft, int &falseCounter);

// GET ULTRASONIC/IR-SENSOR VALUES:
const int MAXCOUNT = 3;
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

        if (sharedMemory && sharedMemory->valid()) { 
            clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << endl;
            
            int mode;

            od4.dataTrigger(2005, [&od4, &mode](cluon::data::Envelope &&envelope) {
              DriveMode currentDriveMode = cluon::extractMessage<DriveMode>(std::move(envelope));
              mode = currentDriveMode.mode();
            });

            //int mode = 0;
            countCars ccars;

            ////**COUNTING CARS:**////
            int amountOfCars = 0; // should be 0, only 3 for testing count passing cars
            int leftCar = 0;
            int frontCar = 0;
            int rightCar = 0;
            // ultrasonic:
            bool gotNewDataFromLeft;
            int falseCounter = 0;
            // float average = 0;
            float frontTotalSum = 0;
            float leftTotalSum = 0;
            int frontCounter = 0;
            int leftCounter = 0;
            float frontSensorValue = 0.0;
            float leftSensorValue = 0.0;
            
            std::array<bool, 3> trafficRules;
            TrafficRules trafficSignRules;
            opendlv::proxy::PedalPositionRequest pedalReq;

                            DriveMode driveMode;
                driveMode.directionInstruction(false); //drivemode initially set to false because we're not ready to receive instruction
                driveMode.atStopSign(false);
                od4.send(driveMode);

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

                if(mode == 0){
                    pedalReq.position(0.13f);
                    od4.send(pedalReq);
                    ssd.run(img , VERBOSE, VIDEO);
                    trafficRules = ShapeDetection(img, VERBOSE, VIDEO);

                    if(ssd.Threshhold_reached){ 
                        cout << "at stop sign" << endl;

                        trafficSignRules.leftAllowed(trafficRules[0]);
                        trafficSignRules.forwardAllowed(trafficRules[1]);
                        trafficSignRules.rightAllowed(trafficRules[2]);
                        od4.send(trafficSignRules); 
                                              
                        driveMode.atStopSign(true);
                        od4.send(driveMode);
                        //mode = 1; //TODO delete this after the od4 is tested
                    }
                    //TODO add linearacceleration
                    //TODO add calibration

                    leftCar = ccars.findCars(greenInputImage, 0, leftCar);
                    amountOfCars = leftCar + frontCar + rightCar;
                } else {
                    pedalReq.position(0.0f);
                    od4.send(pedalReq);
                    bool runOnce = true;
                    if(runOnce){
                        frontCar = ccars.findCars(greenInputImage, 1, frontCar);
                        rightCar = ccars.findCars(greenInputImage, 2, rightCar);
                        runOnce = false;
                    }
                    if (amountOfCars == 0) {
                        driveMode.directionInstruction(true);
                        od4.send(driveMode);
                    } else {
                        auto onDistanceReading{
                            [&od4, &gotNewDataFromLeft,&leftSensorValue, &frontSensorValue,&frontTotalSum, &frontCounter, &leftCounter,&leftTotalSum, &falseCounter](cluon::data::Envelope &&envelope) {
                            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                            const uint16_t senderStamp = envelope.senderStamp(); // Local variables are not available
                            gotNewDataFromLeft = false;
                                if (senderStamp == 0) {
                                    frontSensorValue = getSensorData(msg.distance(), 0, frontTotalSum, frontCounter,gotNewDataFromLeft, falseCounter);
                                } else if (senderStamp == 1) {
                                    leftSensorValue = getSensorData(msg.distance(), 1, leftTotalSum, leftCounter,gotNewDataFromLeft, falseCounter);
                                }
                            }
                        };
                        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(),onDistanceReading);
                        //  **Credit: --->this code is based on example_control code, end*

                        // count how many cars pass by and remove from frontCounter
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

  if (sendStamp == 1) { // left sensor
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
  return -1;
}