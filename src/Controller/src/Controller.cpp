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
    const uint16_t CID{static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};
    const float SPEED{static_cast<float>(stof(commandlineArguments["speed"]))};
    const float leftangle{static_cast<float>(stof(commandlineArguments["langle"]))};
    const float rightangle{static_cast<float>(stof(commandlineArguments["rangle"]))};
    const float leftdelay{static_cast<float>(stof(commandlineArguments["ldelay"]))};
    const float rightdelay{static_cast<float>(stof(commandlineArguments["rdelay"]))};

    cluon::OD4Session od4{CID};

    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
    bool TurnLeft, TurnRight, GoForward;
    bool directionInstructionMode = false;

    double lastCalib = 0;


    //: recive trafic sign rules.
    od4.dataTrigger(2003, [&TurnLeft, &TurnRight, &GoForward](cluon::data::Envelope &&envelope) {
      cout << "RECEIVED TRAFFIC SIGN MESSAGE" << endl;
      TrafficRules trafficSignRules =
          cluon::extractMessage<TrafficRules>(std::move(envelope));

      TurnLeft = trafficSignRules.leftAllowed();
      TurnRight = trafficSignRules.rightAllowed();
      GoForward = trafficSignRules.forwardAllowed();
      if(TurnLeft) cout << "CAN DRIVE LEFT" << endl;
      if(GoForward) cout << "CAN DRIVE FORWARD" << endl;
      if(TurnRight) cout << "CAN DRIVE RIGHT" << endl;
    });


    //: Driving out of the intersection
    od4.dataTrigger(2001, [&od4, &TurnLeft, &TurnRight, &GoForward, &pedalReq,
                          &steerReq, &directionInstructionMode, &rightangle, &SPEED, &leftangle, &leftdelay, &rightdelay](cluon::data::Envelope &&envelope) {
      DirectionInstruction receivedMsg =
          cluon::extractMessage<DirectionInstruction>(std::move(envelope));
      DirectionResponse responseMsg;
      std::cout << "RECEIVED DIRECTION: " << receivedMsg.direction()
                << std::endl;
      if (directionAllowed(receivedMsg.direction(), TurnLeft, TurnRight,
                          GoForward) && directionInstructionMode) {
        std::cout << "ALLOWED. DRIVING NOW" << endl;
        responseMsg.response("Direction allowed");

        //float speed = 0.12f;
        //float initialBoostSpeed = 0.2f;
        float stop = 0.0f;
        //float leftSteerAngle = 0.11f;
        //float rightSteerAngle = -0.4f;

        //float forwardSteerAngle = -0.109f;
        //uint16_t rightDelay = 3300;
        //uint16_t otherDirectionDelay = 1900;
        //uint16_t boostDelay = 5;
        //** Initial boost to car **//
        /*pedalReq.position(initialBoostSpeed);
        od4.send(pedalReq);
        std::this_thread::sleep_for(std::chrono::milliseconds(boostDelay));
        pedalReq.position(stop);
        steerReq.groundSteering(stop);
        od4.send(steerReq);
        od4.send(pedalReq);*/
        //** End initial boost **//

        pedalReq.position(SPEED);
        std::string direction = receivedMsg.direction();
        float steer = direction == "left" ? leftangle : direction == "right" ? rightangle ? 0.0f;
        //cout << "SPEED: " << pedalReq.position() << endl;
        steerReq.groundSteering(steer);
        cout << "STEER: " << steerReq.groundSteering() << endl;
        od4.send(steerReq);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        od4.send(pedalReq);
        int delay = direction == "right" ? rightdelay : leftdelay;
        cout << "DELAY: " << delay << endl;
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

     //bool runOnce = false;
     bool afterStopSign = false;

    //: Setting the drive-mode
    od4.dataTrigger(2005, [&od4, &pedalReq, &SPEED, &afterStopSign](cluon::data::Envelope &&envelope) {
      DriveMode currentDriveMode = cluon::extractMessage<DriveMode>(std::move(envelope));
          bool atStopSign = currentDriveMode.atStopSign(); //e.g at stop sign
          //directionInstructionMode = currentDriveMode.directionInstruction();
          cout << "AFTER STOP SIGN? " << afterStopSign << endl;
          if(atStopSign && !afterStopSign){
            cout << "ready for instruction" << endl;
            pedalReq.position(0.0f);
            /*if(!runOnce){
              pedalReq.position(0.0f);
              runOnce = true;
            }*/
            afterStopSign = true;
            currentDriveMode.mode(1);
          } else if(!atStopSign && !afterStopSign) {
            cout << "Not ready for instruction" << endl;
            pedalReq.position(SPEED);
            currentDriveMode.mode(0);
          }
          od4.send(pedalReq);
          od4.send(currentDriveMode);
    });

    od4.dataTrigger(2007, [&od4, &directionInstructionMode](cluon::data::Envelope &&envelope) {
      InstructionMode instructionMode = cluon::extractMessage<InstructionMode>(std::move(envelope));
      directionInstructionMode = instructionMode.directionAllowed();
      if(directionInstructionMode) cout << "INSTRUCTION ALLOWED " << endl;
    });

    od4.dataTrigger(2008, [&od4 , &pedalReq, &SPEED](cluon::data::Envelope &&envelope) {
      LeadCarDistance leadCarDistance = cluon::extractMessage<LeadCarDistance>(std::move(envelope));
      if(leadCarDistance.distance() == "too close"){
        pedalReq.position(0.0f);
      } else {
        pedalReq.position(SPEED);
      }
      od4.send(pedalReq);
    });

    od4.dataTrigger(2006, [&od4, &steerReq, &lastCalib](cluon::data::Envelope &&envelope) {
      CalibrateSteering calibrateSteering = cluon::extractMessage<CalibrateSteering>(std::move(envelope));
      if (calibrateSteering.CalibrateSteeringAngle() != lastCalib) {
        lastCalib = calibrateSteering.CalibrateSteeringAngle();
        steerReq.groundSteering(calibrateSteering.CalibrateSteeringAngle());
        od4.send(steerReq);
        std::cout << "Calibrate Steering Angle: " << calibrateSteering.CalibrateSteeringAngle() << std::endl;
      }
    });


    while (od4.isRunning()) {
    }
  }
  return retCode;
}

bool directionAllowed(std::string direction, bool allowedLeft, bool allowedRight, bool allowedForward) {
  if (direction == "left")
    return allowedLeft;
  if (direction == "right")
    return allowedRight;
  if (direction == "forward")
    return allowedForward;
  return false;
}
