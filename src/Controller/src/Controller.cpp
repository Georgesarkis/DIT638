#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <stdio.h>
#include <string>

using namespace std;

bool directionAllowed(std::string, bool, bool, bool);

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ){
        cerr << "         --cid:    CID of the OD4Session to send and receive messages" << endl;
    } else {
        const uint16_t CID{static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};

        cluon::OD4Session od4{CID};

        opendlv::proxy::PedalPositionRequest pedalReq;
        opendlv::proxy::GroundSteeringReading steerReq;
        bool TurnLeft, TurnRight, GoForward;

        od4.dataTrigger(2003, [&TurnLeft, &TurnRight, &GoForward](cluon::data::Envelope &&envelope) { 
            TrafficRules trafficSignRules = cluon::extractMessage<TrafficRules>(std::move(envelope));
            
            TurnLeft = trafficSignRules.leftAllowed();
            TurnRight = trafficSignRules.rightAllowed();
            GoForward = trafficSignRules.forwardAllowed();

            // THIS SECTION FOR TESTING. SETTING VARIABLES WILL BE ENOUGH FOR THIS TRIGGER
            if(!trafficSignRules.leftAllowed()){
                cout << "LEFT" << endl;
            } 
            if (!trafficSignRules.rightAllowed()) {
                cout << "RIGHT" << endl;
            } 
            if (!trafficSignRules.forwardAllowed()){
                cout << "FORWARD" << endl;
            }
        });

        // pedalReq and steerReq could be local unless needed to be passed to other code like drive and calibration
        od4.dataTrigger(2001, 
            [&od4, &TurnLeft, &TurnRight, &GoForward, &pedalReq, &steerReq](cluon::data::Envelope &&envelope) { 
                DirectionInstruction receivedMsg = cluon::extractMessage<DirectionInstruction>(std::move(envelope));
                DirectionResponse responseMsg;
                std::cout << "RECEIVED DIRECTION: " << receivedMsg.direction() << std::endl;
                if(directionAllowed(receivedMsg.direction(), TurnLeft, TurnRight, GoForward)){
                    std::cout << "ALLOWED. DRIVING NOW" << endl;
                    responseMsg.response("Direction allowed");
                    //*** THIS SECTION WILL PROBABLY BE SWAPPED WITH THE DRIVE 
                    //*** FUNCTION LATER.
                    pedalReq.position(0.2f);
                    float steer = TurnLeft ? -0.2f : TurnRight ? 0.2f : 0;
                    steerReq.groundSteering(steer);
                    od4.send(steerReq);
                    od4.send(pedalReq);
                    //*****************************************************//
                } else {
                    cout << "NOT ALLOWED" << endl;
                    responseMsg.response("Direction not allowed");
                }
                od4.send(responseMsg);
            });

        od4.dataTrigger(2004, 
            [&od4, &pedalReq](cluon::data::Envelope &&envelope) { 
                LinearAcceleration linAccSpeed = cluon::extractMessage<LinearAcceleration>(std::move(envelope));
                std::cout << "Linear SPEED: " << linAccSpeed.speed() << std::endl;
                pedalReq.position(linAccSpeed.speed());
                od4.send(pedalReq);
            });
        
        od4.dataTrigger(2005, 
            [&od4](cluon::data::Envelope &&envelope) { 
                DriveMode currentDriveMode = cluon::extractMessage<DriveMode>(std::move(envelope));
                //std::cout << "DRIVE MODE: " << currentDriveMode.followLead() << std::endl;
                if(currentDriveMode.followLead()){
                    // CHECK FOR LEFT LANE CAR
                } else if (currentDriveMode.stopSign()){
                    // CHECK FOR RIGHT AND OPPOSITE LANE CAR
                }
            });

        od4.dataTrigger(2006, 
            [&od4 , &steerReq](cluon::data::Envelope &&envelope) { 
                CalibrateSteering calibrateSteering = cluon::extractMessage<CalibrateSteering>(std::move(envelope));
                if(calibrateSteering.CalibrateSteeringAngle() != 0){
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

bool directionAllowed(std::string direction, bool allowedLeft, bool allowedRight, bool allowedForward){
    if(direction == "left") return allowedLeft;
    if(direction == "right") return allowedRight;
    if(direction == "forward") return allowedForward;
    return false;
}
