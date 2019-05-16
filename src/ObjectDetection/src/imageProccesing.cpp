// /*

// Class for Sharing common images between functions.

// */

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

// #include <cstdint>
// #include <iostream>
// #include <memory>
// #include <mutex>

// using namespace cv;
// using namespace std;

// class imageProccesing {

//   Mat temp{};
//   Mat temp2{};

// public:
//   Mat rawImage{};
//   Mat cutImage{};
//   Mat rightSideImage{};
//   Mat rightSideImageGreyScaled{};
//   Mat grayScale{};
//   Mat greenInterval{};
//   Mat blackInterval{};
//   Mat pinkInterval{};
//   Mat orangeInterval{};

//   //: Return a image but without the lowe half , which contains our own car
//   Mat getCutImage(Mat image) {
//     Rect myROI1(0, 0, image.size().width, image.size().height * 3 / 4);
//     return image(myROI1);
//   }

//   Mat getRightSideImage(Mat image) {
//     Rect myROI2(image.size().width / 2, 0, image.size().width / 2,
//                 image.size().height);
//     return image(myROI2);
//   }

//   Mat getGreyScale(Mat image) {
//     cvtColor(image, temp, COLOR_BGR2GRAY);
//     return temp;
//   }

//   Mat getGreenInterval(Mat image) {
//     inRange(image, Scalar(30, 80, 125), Scalar(55, 255, 255), temp);
//     return temp;
//   }

//   Mat getBlackInterval(Mat image) {
//     inRange(image, Scalar(0, 0, 0), Scalar(180, 255, 30), temp);
//     return temp;
//   }

//   Mat getPinkInterval(Mat image) {
//     inRange(image, Scalar(153, 131, 102), Scalar(2, 80, 85), temp);
//     inRange(image, Scalar(0, 0, 0), Scalar(180, 255, 30), temp2);
//     return temp + temp2;
//   }

//   Mat getOrangeInterval(Mat image) {
//     inRange(image, Scalar(2, 130, 154), Scalar(23, 166, 255), temp);
//     return temp;
//   }

//   void setImage(Mat image) {
//     cutImage = getCutImage(image);
//     rightSideImage = getRightSideImage(cutImage);
//     rightSideImageGreyScaled = getGreyScale(rightSideImage);
//     greenInterval = getGreenInterval(image);
//     blackInterval = getBlackInterval(image);
//     pinkInterval = getPinkInterval(image);
//     orangeInterval = getOrangeInterval(image);
//   }
// };
