#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <stdio.h>
#include <string>

using namespace std;

bool directionAllowed(std::string, bool, bool, bool);

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid"))) {
    cerr << "--cid:    CID of the OD4Session to send and receive messages" << endl;
  } else {
    const uint16_t CID{static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};
    const float SPEED = 0.109f;
    cluon::OD4Session od4{CID};

    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
    bool TurnLeft, TurnRight, GoForward;
    bool directionInstructionMode = false;
    bool runOnce = true;

    //: Driving out of the intersection
    od4.dataTrigger(2001, [&od4, &TurnLeft, &TurnRight, &GoForward, &pedalReq, &steerReq, &directionInstructionMode, &SPEED](cluon::data::Envelope &&envelope) {
      DirectionInstruction receivedMsg = cluon::extractMessage<DirectionInstruction>(std::move(envelope));
      DirectionResponse responseMsg;
      std::cout << "RECEIVED DIRECTION: " << receivedMsg.direction() << std::endl;
      if (directionAllowed(receivedMsg.direction(), TurnLeft, TurnRight, GoForward) && directionInstructionMode) {
        std::cout << "ALLOWED. DRIVING NOW" << endl;
        responseMsg.response("Direction allowed");

        float initialBoostSpeed = 0.2f;
        float stop = 0.0f;

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

        pedalReq.position(SPEED);
        std::string direction = receivedMsg.direction();
        float steer = direction == "left" ? 0.12f : direction == "right" ? -0.4f : 0.0f;
        steerReq.groundSteering(steer);
        od4.send(steerReq);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        od4.send(pedalReq);
        int delay = direction == "right" ? 4000 : 7000;
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

    //: receive trafic sign rules.
    od4.dataTrigger(2003, [&TurnLeft, &TurnRight, &GoForward](cluon::data::Envelope &&envelope) {
      cout << "RECEIVED TRAFFIC SIGN MESSAGE" << endl;
      TrafficRules trafficSignRules = cluon::extractMessage<TrafficRules>(std::move(envelope));

      TurnLeft = trafficSignRules.leftAllowed();
      TurnRight = trafficSignRules.rightAllowed();
      GoForward = trafficSignRules.forwardAllowed();
      if(TurnLeft) cout << "CAN DRIVE LEFT" << endl;
      if(GoForward) cout << "CAN DRIVE FORWARD" << endl;
      if(TurnRight) cout << "CAN DRIVE RIGHT" << endl;
    });

     bool afterStopSign = false;

    //: Setting the drive-mode
    od4.dataTrigger(2005, [&od4, &pedalReq, &SPEED, &afterStopSign](cluon::data::Envelope &&envelope) {
      DriveMode currentDriveMode = cluon::extractMessage<DriveMode>(std::move(envelope));
          bool atStopSign = currentDriveMode.atStopSign(); //e.g at stop sign
          if(atStopSign && !afterStopSign){
            cout << "at stop sign" << endl;
            pedalReq.position(0.0f);
            afterStopSign = true;
            currentDriveMode.mode(1);
          } else if(!atStopSign && !afterStopSign) {
            cout << "driving to stop sign" << endl;
            pedalReq.position(SPEED);
            currentDriveMode.mode(0);
          }
          od4.send(pedalReq);
          od4.send(currentDriveMode);
    });

    //: Sets whether instruction is ready to be received
    od4.dataTrigger(2007, [&od4, &directionInstructionMode, &runOnce](cluon::data::Envelope &&envelope) {
      InstructionMode instructionMode = cluon::extractMessage<InstructionMode>(std::move(envelope));
      directionInstructionMode = instructionMode.directionAllowed();
      if(directionInstructionMode){
        if(runOnce) cout << "INSTRUCTION ALLOWED" << endl;
        runOnce = false;
      } 
    });

    //: ACC follow the lead car
    od4.dataTrigger(2008, [&od4 , &pedalReq, &SPEED](cluon::data::Envelope &&envelope) {
      LeadCarDistance leadCarDistance = cluon::extractMessage<LeadCarDistance>(std::move(envelope));
      if(leadCarDistance.distance() == "too close"){
        pedalReq.position(0.0f);
      } else {
        pedalReq.position(SPEED);
      }
      od4.send(pedalReq);
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
