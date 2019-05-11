#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>


using namespace cv;
using namespace std;

Point MiddleOfRectangle;

class leadCarScan
{
private:
  double CalibrateSteering(cv::Mat img, cv::Point point, bool VERBOSE);

public:
  double findLeadCar(const Mat &greenIntervalOutput, bool VIDEO);
  double CalibrateSteeringAngle(double areaOfContour, Mat img, bool VERBOSE);
};

double leadCarScan::findLeadCar(const Mat &greenIntervalOutput, bool VIDEO)
{
  double foundArea = 0;
  Rect BigestBoundRect;
  vector<vector<Point>> contours;
  findContours(greenIntervalOutput, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  vector<vector<Point>> contoursPoly(contours.size());
  vector<Rect> boundRect(contours.size());

  for (size_t i = 0; i < contours.size(); i++)
  {
    approxPolyDP(contours[i], contoursPoly[i], 3, true);
    boundRect[i] = boundingRect(contoursPoly[i]);
  }

  Mat drawing = Mat::zeros(greenIntervalOutput.size(), CV_8UC3);
  for (unsigned int i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar(0, 0, 255);

    drawContours(drawing, contoursPoly, i, color, 2, 4,
                 vector<Vec4i>(), 0);
    rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 4,
              0);

    if (contourArea(contours[i]) > foundArea)
    {
      foundArea = contourArea(contours[i]);
      BigestBoundRect = boundingRect(contoursPoly[i]);
    }
  }

  Scalar red = Scalar(0, 0, 255);
  Scalar green = Scalar(0, 255, 0);
  rectangle(greenIntervalOutput, BigestBoundRect.tl(), BigestBoundRect.br(), red, 2, 4, 0);
  MiddleOfRectangle = Point(BigestBoundRect.tl() * 0.5 + BigestBoundRect.br() * 0.5);
  circle(greenIntervalOutput, MiddleOfRectangle, 3, green);

  if (VIDEO)
    imshow("greenIntervalOutput", greenIntervalOutput);

  return foundArea;
}

double leadCarScan::CalibrateSteeringAngle(double areaOfContour, Mat img, bool VERBOSE)
{
  double Angle;
  if (areaOfContour > 1000)
  {
    return CalibrateSteering(img, MiddleOfRectangle, VERBOSE);
  }
  return Angle = 0;
}

double leadCarScan::CalibrateSteering(cv::Mat img, cv::Point point, bool VERBOSE)
{
  double leftAngle = 0.11;
  double rightAngle = -0.11;

  if (point.x < (img.size().width / 2) - 50)
  {
    if (VERBOSE)
      cout << "turn car to left" << endl;
    return leftAngle;
  }
  else if (point.x > (img.size().width / 2) + 50)
  {
    if (VERBOSE)
      cout << "turn car to right" << endl;
    return rightAngle;
  }
  return 0;
}