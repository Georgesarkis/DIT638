#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <stdio.h>
#include <string>
#include <array>

#include "SignDetection.hpp"
#include "StopSignDetection.cpp"
#include "linearAcceleration.cpp"
#include "countCars.hpp"
#include "leadCarScan.cpp"

using namespace std;
using namespace cv;

Mat GetCroppedImage(Mat img);
array<bool, 3> ShapeDetection(Mat img, bool VERBOSE);

//****CLASSES:****
StopSignDetection ssd;
leadCarScan leadCar;
countCars ccars;

//COUNTING CARS: (where to move these since they are used in the functions?)
int amountOfCars = 0;    //should be 0
int maxAmountOfCars = 3; //not used
int mode = 0;
int leftCar = 0;
int rightCar = 0;
int frontCar = 0;

Mat getInterval(Mat img, string color)
{
  Mat hsvImg;

  cvtColor(img, hsvImg, COLOR_BGR2HSV);
  Mat intervalOutput;

  if (color == "black")
  { //black
    inRange(hsvImg, Scalar(0, 0, 0), Scalar(180, 255, 30), intervalOutput);
  }
  if (color == "orange")
  { //orange
    inRange(hsvImg, Scalar(2, 130, 154), Scalar(23, 166, 255), intervalOutput);
  }
  else if (color == "green")
  { //green
    inRange(hsvImg, Scalar(30, 80, 125), Scalar(55, 255, 255), intervalOutput);
  }
  return intervalOutput;
}

bool scanForStopSign(Mat img, bool VERBOSE, bool VIDEO)
{
  if (ssd.Threshhold_reached == false)
  {
    ssd.run(img, VERBOSE, VIDEO);
  }
  else
  {
    // cout << "-- Exiting stopsign detection."
    //       << " -- Failed frames: " << ssd.failed_frames << endl;
  }
  return ssd.Threshhold_reached;
}

void scanForCarInLeft(const Mat &image)
{
  leftCar = ccars.findCars(image, 0, leftCar);
}

void scanForCarInFront(const Mat &image)
{
  frontCar = ccars.findCars(image, 1, frontCar);
}

void scanForCarInRight(const Mat &image)
{
  rightCar = ccars.findCars(image, 2, rightCar);
}

void scanForPassingCars(float sensorDistance, int currentAmountOfCars, int sensorType, const Mat &image)
{
  amountOfCars = ccars.countPassingCars(sensorDistance, currentAmountOfCars, sensorType, image);
}

//: ACC
//: scan for the car to get the contour
double areaOfContour;

std::array<bool, 3> scanForTrafficSigns(Mat img, bool VERBOSE)
{
  std::array<bool, 3> trafficRules = ShapeDetection(img, VERBOSE);
  return trafficRules;
}

void listenToCommand() {}

int32_t main(int32_t argc, char **argv)
{

  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height")))
  {
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
    cerr << "         --height: height of the frame" << endl;
    cerr << "         --video: indicates whether the video feed should be "
            "displayed"
         << endl;
    cerr << "         --carspeed: the speed of the car" << endl;
    cerr << "         --nrsign: the number of stopsigns the car is allowed to "
            "miss, should be 1-3"
         << endl;
    cerr << "         --minarea: the minumum area of stopsign" << endl;
  }
  else
  {

    DriveMode driveMode;

    const string NAME{commandlineArguments["name"]};
    const uint32_t WIDTH{
        static_cast<uint32_t>(stoi(commandlineArguments["width"]))};
    const uint32_t HEIGHT{
        static_cast<uint32_t>(stoi(commandlineArguments["height"]))};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const bool VIDEO{commandlineArguments.count("video") != 0};
    float CARSPEED = 0.1;
    int MINAREA = 1000;
    int NRSIGN = 5;
    // Attach to the shared memory.
    unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
    if (sharedMemory && sharedMemory->valid())
    {
      clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name()
           << " (" << sharedMemory->size() << " bytes)." << endl;

      // Interface to a running OpenDaVINCI session; here, you can send and
      // receive messages.
      cluon::OD4Session od4{
          static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning())
      {
        Mat img;
        sharedMemory->wait();
        sharedMemory->lock();
        {
          Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
          img = wrapped.clone();
        }
        sharedMemory->unlock();

        TrafficRules trafficSignRules;
        CalibrateSteering calibrateSteering;
        bool stopSignFound;
        std::array<bool, 3> trafficRules;
        double AreaOfContour;

        Mat greenInputImage = getInterval(img, "green");
        Mat blackInputImage = getInterval(img, "black");
        Mat orangeInputImage = getInterval(img, "orange");

        switch (mode)
        {
        case 0:
          stopSignFound = scanForStopSign(img, VERBOSE, VIDEO);
          driveMode.stopSign(stopSignFound);
          driveMode.followLead(!stopSignFound);
          od4.send(driveMode);

          scanForCarInLeft(greenInputImage);

          //calibrating steering angle
          AreaOfContour = leadCar.findLeadCar(orangeInputImage, VIDEO);
          calibrateSteering.CalibrateSteeringAngle(leadCar.CalibrateSteeringAngle(AreaOfContour, orangeInputImage, VERBOSE));
          od4.send(calibrateSteering);

          trafficRules = scanForTrafficSigns(img, VERBOSE);
          if (!trafficRules[0])
            cout << "TRAFFIC LEFT " << endl;
          if (!trafficRules[1])
            cout << "TRAFFIC FORWARD" << endl;
          if (!trafficRules[2])
            cout << "TRAFFIC RIGHT" << endl;
          trafficSignRules.leftAllowed(trafficRules[0]);
          trafficSignRules.forwardAllowed(trafficRules[1]);

          trafficSignRules.rightAllowed(trafficRules[2]);
          od4.send(trafficSignRules);

          //CALCULATE CARS:
          amountOfCars = leftCar + frontCar + rightCar;
          break;

        case 1:
          scanForCarInFront(greenInputImage);
          scanForCarInRight(greenInputImage);
          // listenToCommand();

          //CALCULATE CARS:
          amountOfCars = leftCar + frontCar + rightCar;
          break;

        case 2:
          if (amountOfCars == 0)
          {
            //drive out of intersection routine
          }
          else
          {
            // scanForPassingCars(blackInputImage);
          }
          break;
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}