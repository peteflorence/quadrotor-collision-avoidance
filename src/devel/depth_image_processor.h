#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>


class DepthImageProcessor {
public:
  void TestOpenCV(Eigen::Quaternionf const& q_b, cv::Point & corner1, cv::Point & corner2);



private:
  
  double v = 6.0;


};