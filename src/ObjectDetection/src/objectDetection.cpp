#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <array>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <stdio.h>
#include <string>

#include "SignDetection.hpp"
#include "StopSignDetection.cpp"
#include "countCars.hpp"
#include "imageProccesing.cpp"
#include "leadCarScan.cpp"

using namespace std;
using namespace cv;

Mat GetCroppedImage(Mat img);
array<bool, 3> ShapeDetection(Mat img, bool VERBOSE, bool VIDEO);

//****CLASSES:****
StopSignDetection ssd;
leadCarScan leadCar;
countCars ccars;
imageProccesing imgProc;

int mode = 0;

bool scanForStopSign(Mat img, bool VERBOSE, bool VIDEO) {
  if (ssd.Threshhold_reached == false) {
    ssd.run(img, VERBOSE, VIDEO);
  } else {
    // cout << "-- Exiting stopsign detection."
    //       << " -- Failed frames: " << ssd.failed_frames << endl;
  }
  return ssd.Threshhold_reached;
}

// GET ULTRASONIC/IR-SENSOR VALUES:
const int MAXCOUNT = 3;
float frontSensorData[MAXCOUNT];
float leftSensorData[MAXCOUNT];

float getSensorData(float distanceMessage, int sendStamp, float &totalSum,
                    int &counter, bool &gotNewDataFromLeft, int &falseCounter);

int scanForCarInLeft(const Mat &image, int leftCar) {
  return ccars.findCars(image, 0, leftCar);
}

int scanForCarInFront(const Mat &image, int frontCar) {
  return ccars.findCars(image, 1, frontCar);
}

int scanForCarInRight(const Mat &image, int rightCar) {
  return ccars.findCars(image, 2, rightCar);
}

int scanForPassingCars(float sensorDistance, int currentAmountOfCars,
                       int sensorType, Mat image) {
  return ccars.countPassingCars(sensorDistance, currentAmountOfCars, sensorType,
                                image);
}

//: ACC
//: scan for the car to get the contour
double areaOfContour;
std::array<bool, 3> scanForTrafficSigns(Mat img, bool VERBOSE, bool VIDEO) {
  std::array<bool, 3> trafficRules = ShapeDetection(img, VERBOSE, VIDEO);
  return trafficRules;
}

int32_t main(int32_t argc, char **argv) {

  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height"))) {
    cerr << argv[0]
         << " attaches to a shared memory area containing an ARGB image."
         << endl;
    cerr << "Usage:   " << argv[0]
         << " --cid=<OD4 session> --name=<name of shared memory area> "
            "[--verbose]"
         << endl;
    cerr << "         --cid:    CID of the OD4Session to send and receive "
            "messages"
         << endl;
    cerr << "         --name:   name of the shared memory area to attach"
         << endl;
    cerr << "         --width:  width of the frame" << endl;
    cerr << "         --height: height of the frame" << endl;
    cerr << "         --video: indicates whether the video feed should be "
            "displayed"
         << endl;
  } else {

    DriveMode driveMode;

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

    const string NAME{commandlineArguments["name"]};
    const uint32_t WIDTH{
        static_cast<uint32_t>(stoi(commandlineArguments["width"]))};
    const uint32_t HEIGHT{
        static_cast<uint32_t>(stoi(commandlineArguments["height"]))};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const bool VIDEO{commandlineArguments.count("video") != 0};
    // double CARSPEED = 0.1;
    // int MINAREA = 1000;
    // int NRSIGN = 5;
    // Attach to the shared memory.
    unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
    if (sharedMemory && sharedMemory->valid()) {
      clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name()
           << " (" << sharedMemory->size() << " bytes)." << endl;

      // Interface to a running OpenDaVINCI session; here, you can send and
      // receive messages.
      cluon::OD4Session od4{
          static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};

      driveMode.directionInstruction(false);
      od4.send(driveMode);

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning()) {
        Mat img;
        sharedMemory->wait();
        sharedMemory->lock();
        {
          Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
          img = wrapped.clone();
        }
        sharedMemory->unlock();

        LinearAcceleration linAcc;
        TrafficRules trafficSignRules;
        CalibrateSteering calibrateSteering;
        bool stopSignFound;
        std::array<bool, 3> trafficRules;
        double AreaOfContour;

        //: preforms imgage proccesing for the images , so the fucntions can use
        // commonly used images, like the croped image
        imgProc.setImage(img);

        switch (mode) {
        case 0:
          stopSignFound =
              scanForStopSign(imgProc.rightSideImageGreyScaled, VERBOSE, VIDEO);
          if(!stopSignFound) mode = 1;
          leftCar = scanForCarInLeft(imgProc.greenInterval, leftCar);

          // calibrating steering angle
          AreaOfContour = leadCar.findLeadCar(imgProc.orangeInterval, VIDEO);
          calibrateSteering.CalibrateSteeringAngle(
              leadCar.CalibrateSteeringAngle(AreaOfContour,
                                             imgProc.orangeInterval, VERBOSE));
          od4.send(calibrateSteering);

          //: ACC
          // cout << "Lead car contour: " << AreaOfContour << endl;
          linAcc.contourArea(AreaOfContour);
          od4.send(linAcc);

          trafficRules = scanForTrafficSigns(img, VERBOSE, VIDEO);
          trafficSignRules.leftAllowed(trafficRules[0]);
          trafficSignRules.forwardAllowed(trafficRules[1]);
          trafficSignRules.rightAllowed(trafficRules[2]);
          od4.send(trafficSignRules);

          // CALCULATE CARS:
          amountOfCars = leftCar + frontCar + rightCar;
          break;

        case 1:
          frontCar = scanForCarInFront(imgProc.greenInterval, frontCar);
          rightCar = scanForCarInRight(imgProc.greenInterval, rightCar);

          // CALCULATE CARS:
          amountOfCars = leftCar + frontCar + rightCar;
          if (amountOfCars == 0) {
            driveMode.directionInstruction(true);
            od4.send(driveMode);
            // drive out of intersection routine
          } else {
            auto onDistanceReading{[VERBOSE, &od4, &gotNewDataFromLeft,
                                    &leftSensorValue, &frontSensorValue,
                                    &frontTotalSum, &frontCounter, &leftCounter,
                                    &leftTotalSum, &falseCounter](
                                       cluon::data::Envelope &&envelope) {
              auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(
                  std::move(envelope));
              const uint16_t senderStamp =
                  envelope.senderStamp(); // Local variables are not available

              gotNewDataFromLeft = false;

              if (senderStamp == 0) {
                frontSensorValue = getSensorData(
                    msg.distance(), 0, frontTotalSum, frontCounter,
                    gotNewDataFromLeft, falseCounter);
              } else if (senderStamp == 1) {
                leftSensorValue =
                    getSensorData(msg.distance(), 1, leftTotalSum, leftCounter,
                                  gotNewDataFromLeft, falseCounter);
              }
            }};
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(),
                            onDistanceReading);
            //  **Credit: --->this code is based on example_control code, end*

            // count how many cars pass by and remove from frontCounter
            amountOfCars = scanForPassingCars(frontSensorValue, amountOfCars, 0,
                                              imgProc.blackInterval);

            if (!gotNewDataFromLeft) {
              falseCounter++;
            }
            if (falseCounter >= 5) { // needed for the left sensor
              amountOfCars =
                  scanForPassingCars(0, amountOfCars, 1, imgProc.blackInterval);
            } else {
              amountOfCars = scanForPassingCars(leftSensorValue, amountOfCars,
                                                1, imgProc.blackInterval);
            }
          }
          break;
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}

float getSensorData(float distanceMessage, int sendStamp, float &totalSum,
                    int &counter, bool &gotNewDataFromLeft, int &falseCounter) {

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
