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
        cluon::OD4Session od4(CID,
            [](cluon::data::Envelope &&envelope) noexcept {
                cout << envelope.dataType() << endl;
            if (envelope.dataType() == 2001) {
                DirectionInstruction receivedMsg = cluon::extractMessage<DirectionInstruction>(std::move(envelope));
                std::cout << "Received message: " << receivedMsg.direction() << endl;
                if(receivedMsg.direction() == "left"){
                    
                } else if(receivedMsg.direction() == "right"){

                } else {

                }
            }
        });
        while(od4.isRunning()){

        }
    }
    return retCode;
}