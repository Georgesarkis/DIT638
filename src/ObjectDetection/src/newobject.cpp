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
        cluon::OD4Session od4{
            static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};
        
        while(od4.running){
            Mat img;
            sharedMemory->wait();
            sharedMemory->lock();
            {
            Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
            img = wrapped.clone();
            }
            sharedMemory->unlock();

            ssd.run(img , true, true);
            bool stopSignFound = ssd.Threshold_reached;

            string banana = stopSignFound ? "true" : "false";

            cout << "STUFF: " << banana << endl;
        }
    }
    return 0;
}