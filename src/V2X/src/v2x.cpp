#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <stdio.h>
#include <string>

using namespace std;

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ){
        cerr << "         --cid:    CID of the OD4Session to send and receive messages" << endl;
    } else {
        const uint16_t CID{static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};

        cluon::OD4Session od4{CID};

        //: instruction response from controller
        od4.dataTrigger(2002, [](cluon::data::Envelope &&envelope) { 
            DirectionResponse directionResponseMsg = cluon::extractMessage<DirectionResponse>(std::move(envelope));
            cout << "MESSAGE: " << directionResponseMsg.response() << endl;
            if(directionResponseMsg.response() == "Direction not allowed"){
                cout << directionResponseMsg.response() << ". Try again" << endl;
            } else {
                cout << "Instruction successful" << endl;
            }
        });

        while (od4.isRunning()){
            string instruction;
            cout << "Enter direction: left, right, forward" << endl;
            cin >> instruction;
            if (instruction != "" || instruction == "left" || instruction == "right" || instruction == "forward"){
                DirectionInstruction instructionToSend;
                instructionToSend.direction(instruction);
                cout << "INSTRUCTION: " << instructionToSend.direction() << endl;
                od4.send(instructionToSend);
            } else {
                cout << "INVALID INSTRUCTION!" << endl;
            }
        }
    }
    return retCode;
}