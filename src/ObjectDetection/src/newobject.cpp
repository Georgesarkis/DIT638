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

using namespace std;
using namespace cv;

StopSignDetection ssd;

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
        
        if (sharedMemory && sharedMemory->valid()) { 
            clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << endl;
            
            int mode = 0;

            driveMode.directionInstruction(false);
            od4.send(driveMode);

            od4.dataTrigger(2005, [&od4](cluon::data::Envelope &&envelope) {
                DriveMode currentDriveMode =
                    cluon::extractMessage<DriveMode>(std::move(envelope));
                mode = driveMode.mode();
                cout << "mode: " << mode << endl;
            }
            
            std::array<bool, 3> trafficRules;
            TrafficRules trafficSignRules;

            while(od4.isRunning()){
                Mat img;
                sharedMemory->wait();
                sharedMemory->lock();
                {
                Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                img = wrapped.clone();
                }
                sharedMemory->unlock();

                ssd.run(img , true, true);
                bool stopSignFound = ssd.Threshhold_reached;
                if(!stopSignFound){ 
                    driveMode.directionInstruction(false);
                    od4.send(driveMode);
                }

                trafficRules = scanForTrafficSigns(img, VERBOSE, VIDEO);
                cout << "left: " << trafficRules[0] << endl; 
                cout << "for: " << trafficRules[1] << endl; 
                cout << "right: " << trafficRules[2] << endl; 

                trafficSignRules.leftAllowed(trafficRules[0]);
                trafficSignRules.forwardAllowed(trafficRules[1]);
                trafficSignRules.rightAllowed(trafficRules[2]);
                od4.send(trafficSignRules);
            }
        }
    }
    return 0;
}