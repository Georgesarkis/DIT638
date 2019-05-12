#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <stdio.h>
#include <string>
#include <linearAcceleration.cpp>

using namespace std;

linearAcceleration linearACC;

bool directionAllowed(std::string, bool, bool, bool);

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid"))) {
    cerr << "         --cid:    CID of the OD4Session to send and receive "
            "messages"
         << endl;
  } else {
    const uint16_t CID{
        static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};

    cluon::OD4Session od4{CID};

    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringReading steerReq;
    bool TurnLeft, TurnRight, GoForward;

    od4.dataTrigger(2003, [&TurnLeft, &TurnRight,
                           &GoForward](cluon::data::Envelope &&envelope) {
      TrafficRules trafficSignRules =
          cluon::extractMessage<TrafficRules>(std::move(envelope));

      TurnLeft = trafficSignRules.leftAllowed();
      TurnRight = trafficSignRules.rightAllowed();
      GoForward = trafficSignRules.forwardAllowed();

      // THIS SECTION FOR TESTING. SETTING VARIABLES WILL BE ENOUGH FOR THIS
      // TRIGGER
      if (!trafficSignRules.leftAllowed()) {
        cout << "LEFT" << endl;
      }
      if (!trafficSignRules.rightAllowed()) {
        cout << "RIGHT" << endl;
      }
      if (!trafficSignRules.forwardAllowed()) {
        cout << "FORWARD" << endl;
      }
    });

    od4.dataTrigger(2001, [&od4, &TurnLeft, &TurnRight, &GoForward, &pedalReq,
                           &steerReq](cluon::data::Envelope &&envelope) {
      DirectionInstruction receivedMsg =
          cluon::extractMessage<DirectionInstruction>(std::move(envelope));
      DirectionResponse responseMsg;
      std::cout << "RECEIVED DIRECTION: " << receivedMsg.direction()
                << std::endl;
      if (directionAllowed(receivedMsg.direction(), TurnLeft, TurnRight,
                           GoForward)) {
        std::cout << "ALLOWED. DRIVING NOW" << endl;
        responseMsg.response("Direction allowed");

        float speed = 0.12f;
        float initialBoostSpeed = 0.2f;
        float stop = 0.0f;
        float leftSteerAngle = 0.11f;
        float rightSteerAngle = -0.4f;
        float forwardSteerAngle = -0.04f;
        uint16_t rightDelay = 3300;
        uint16_t otherDirectionDelay = 1900;
        uint16_t boostDelay = 5;
        //** Initial boost to car **//
        pedalReq.position(initialBoostSpeed);
        od4.send(pedalReq);
        std::this_thread::sleep_for(std::chrono::milliseconds(boostDelay));
        pedalReq.position(stop);
        steerReq.groundSteering(stop);
        od4.send(steerReq);
        od4.send(pedalReq);
        //** End initial boost **//

        pedalReq.position(speed);
        float steer = TurnLeft
                          ? leftSteerAngle
                          : TurnRight ? rightSteerAngle : forwardSteerAngle;
        steerReq.groundSteering(steer);
        od4.send(steerReq);
        od4.send(pedalReq);
        int delay = TurnRight ? rightDelay : otherDirectionDelay;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        pedalReq.position(stop);
        steerReq.groundSteering(stop);
        od4.send(steerReq);
        od4.send(pedalReq);
      } else {
        cout << "NOT ALLOWED" << endl;
        responseMsg.response("Direction not allowed");
      }
      od4.send(responseMsg);
    });

    od4.dataTrigger(2004, [&od4, &pedalReq](cluon::data::Envelope &&envelope) {
      LinearAcceleration linAccSpeed =
          cluon::extractMessage<LinearAcceleration>(std::move(envelope));
      double countourArea = linAccSpeed.contourArea();
      std::cout << "Leading Car contour area: " << countourArea << std::endl;

      double carSpeed = linearACC.getSpeed(countourArea);

      std::cout << "Car speed: " << carSpeed << std::endl;
      pedalReq.position((float)carSpeed);
      od4.send(pedalReq);
    });

    od4.dataTrigger(2005, [&od4](cluon::data::Envelope &&envelope) {
      DriveMode currentDriveMode =
          cluon::extractMessage<DriveMode>(std::move(envelope));
      // std::cout << "DRIVE MODE: " << currentDriveMode.followLead() <<
      // std::endl;
      if (currentDriveMode.followLead()) {
        // CHECK FOR LEFT LANE CAR
      } else if (currentDriveMode.stopSign()) {
        // CHECK FOR RIGHT AND OPPOSITE LANE CAR
      }
    });

    od4.dataTrigger(2006, [&od4, &steerReq](cluon::data::Envelope &&envelope) {
      CalibrateSteering calibrateSteering =
          cluon::extractMessage<CalibrateSteering>(std::move(envelope));
      if (calibrateSteering.CalibrateSteeringAngle() < 0 || calibrateSteering.CalibrateSteeringAngle() > 0) {
        steerReq.groundSteering((long)calibrateSteering.CalibrateSteeringAngle());
        od4.send(steerReq);
        std::cout << "Calibrate Steering Angle: "
                  << calibrateSteering.CalibrateSteeringAngle() << std::endl;
      }
    });

    while (od4.isRunning()) {
    }
  }
  return retCode;
}

bool directionAllowed(std::string direction, bool allowedLeft,
                      bool allowedRight, bool allowedForward) {
  if (direction == "left")
    return allowedLeft;
  if (direction == "right")
    return allowedRight;
  if (direction == "forward")
    return allowedForward;
  return false;
}
