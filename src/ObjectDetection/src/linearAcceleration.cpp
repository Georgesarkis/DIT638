#include <string>
#include <iostream>
using namespace std;

class linearAcceleration {

  float map(float x, float in_min, float in_max, float out_min, float out_max);

public:
  float getSpeed(float in);
  float max_thresh;
  float low_thresh;
  float outOfBounds_thresh;
  float car_speed_max = 0.30;
  bool VERBOSE = false;
  void setVERBOSE(bool in);
};

float linearAcceleration::map(float x, float in_min, float in_max,
                              float out_min, float out_max) {
  float out_min_100 = out_min * 100;
  float out_max_100 = out_max * 100;
  float pre_calc =
      (x - in_min) * (out_max_100 - out_min_100) / (in_max - in_min) +
      out_min_100;
  return pre_calc;
}

float linearAcceleration::getSpeed(float in) {

  //: Lead car too close
  if (in <= low_thresh) {

    if (VERBOSE)
      std::cout << "Lead car is close ... Stoping" << endl;

    return 0;

    //: Car is inside threashhold
  } else if ((in < max_thresh) && (in > low_thresh)) {

    if (VERBOSE)
      std::cout << "Lead car inside threashhold, Driving normally" << std::endl;

    return map(in, low_thresh, max_thresh, 0, car_speed_max);

    //: Lead car too far away
  } else if ((in > max_thresh) && (in < outOfBounds_thresh)) {

    if (VERBOSE)
      std::cout << "Lead car is too far away... Driving at max speed"
                << std::endl;

    return car_speed_max;

    //: No lead car found
  } else if (in >= outOfBounds_thresh) {

    if (VERBOSE)
      std::cout << "No lead car found, Driving at half max speed" << std::endl;

    return car_speed_max / 2;

  } else {
    std::cout << "Error Logic Wrong" << std::endl;
  }
}
