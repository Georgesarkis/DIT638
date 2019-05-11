#include <iostream>
#include <string>
using namespace std;

class linearAcceleration {

  double map(double x, double in_min, double in_max, double out_min,
             double out_max);

public:
  double getSpeed(double in);
  double max_thresh;
  double low_thresh;
  double outOfBounds_thresh;
  double car_speed_max;
  bool VERBOSE = false;
  void setVERBOSE(bool in);
};

double linearAcceleration::map(double x, double in_min, double in_max,
                               double out_min, double out_max) {
  double out_min_100 = out_min * 100;
  double out_max_100 = out_max * 100;
  double pre_calc =
      (x - in_min) * (out_max_100 - out_min_100) / (in_max - in_min) +
      out_min_100;
  return pre_calc;
}

double linearAcceleration::getSpeed(double in) {

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
